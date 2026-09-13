"""Hardware-independent runtime boundary for simulation and robot deployment."""

from dataclasses import dataclass
from time import perf_counter

try:
    from typing import Protocol
except ImportError:
    try:
        from typing_extensions import Protocol
    except ImportError:
        Protocol = object

from .models import Matrix3, MotionPrimitive, SelectionResult, Vector3
from .risk import adaptive_risk_threshold
from .selector import PrimitiveSelector


@dataclass(frozen=True)
class RuntimeObservation:
    joint_angles: Vector3
    joint_covariance: Matrix3
    obstacle_distance_m: float
    obstacle_age_s: float = 0.0


@dataclass(frozen=True)
class RuntimeDecision:
    selection: SelectionResult
    latency_ms: float
    command: str


class RobotAdapter(Protocol):
    """Small boundary to replace with a ROS 2 JetMax adapter tomorrow."""

    def execute(self, primitive: MotionPrimitive) -> None: ...

    def hold(self, reason: str) -> None: ...


class SimulatedRobotAdapter:
    def __init__(self, initial_angles: Vector3 = (0.0, 0.0, 0.0)) -> None:
        self.joint_angles = initial_angles
        self.last_command = "READY"
        self.last_reason = ""

    def reset(self, angles: Vector3 = (0.0, 0.0, 0.0)) -> None:
        self.joint_angles = angles
        self.last_command = "READY"
        self.last_reason = ""

    def execute(self, primitive: MotionPrimitive) -> None:
        self.joint_angles = primitive.terminal
        self.last_command = f"EXECUTE {primitive.primitive_id}"
        self.last_reason = ""

    def hold(self, reason: str) -> None:
        self.last_command = "HOLD"
        self.last_reason = reason


class SafetyRuntime:
    def __init__(
        self,
        selector: PrimitiveSelector,
        adapter: RobotAdapter,
        *,
        maximum_obstacle_age_s: float = 0.25,
        start_tolerance_rad: float = 0.03,
    ) -> None:
        if maximum_obstacle_age_s <= 0.0:
            raise ValueError("maximum_obstacle_age_s must be positive")
        if start_tolerance_rad < 0.0:
            raise ValueError("start_tolerance_rad cannot be negative")
        self.selector = selector
        self.adapter = adapter
        self.maximum_obstacle_age_s = maximum_obstacle_age_s
        self.start_tolerance_rad = start_tolerance_rad

    def _hold_result(self, threshold: float, distance_m: float, reason: str) -> SelectionResult:
        return SelectionResult(
            primitive_id=None,
            risk_bound=None,
            threshold=threshold,
            distance_bin_m=max(0.0, distance_m),
            fallback=True,
            reason=reason,
        )

    def step(self, observation: RuntimeObservation, goal: Vector3) -> RuntimeDecision:
        started = perf_counter()
        threshold = adaptive_risk_threshold(
            max(0.0, observation.obstacle_distance_m), observation.joint_covariance
        )

        if observation.obstacle_age_s > self.maximum_obstacle_age_s:
            selection = self._hold_result(
                threshold,
                observation.obstacle_distance_m,
                "obstacle observation is stale",
            )
        else:
            selection = self.selector.select(
                obstacle_distance_m=max(0.0, observation.obstacle_distance_m),
                threshold=threshold,
                goal_joint_angles=goal,
            )

        if selection.fallback:
            self.adapter.hold(selection.reason)
            command = "HOLD"
        else:
            primitive = next(
                primitive
                for primitive in self.selector.library.primitives
                if primitive.primitive_id == selection.primitive_id
            )
            if any(
                abs(actual - expected) > self.start_tolerance_rad
                for actual, expected in zip(observation.joint_angles, primitive.start)
            ):
                selection = self._hold_result(
                    threshold,
                    observation.obstacle_distance_m,
                    "robot is outside the certified primitive start tolerance",
                )
                self.adapter.hold(selection.reason)
                command = "HOLD"
            else:
                self.adapter.execute(primitive)
                command = f"EXECUTE {primitive.primitive_id}"

        return RuntimeDecision(
            selection=selection,
            latency_ms=(perf_counter() - started) * 1000.0,
            command=command,
        )
