"""Shared immutable data contracts used by offline and online components."""

from dataclasses import dataclass
from typing import Optional, Tuple

Vector3 = Tuple[float, float, float]
Matrix3 = Tuple[Vector3, Vector3, Vector3]


@dataclass(frozen=True)
class MotionPrimitive:
    """A sampled, fixed-duration joint-space trajectory."""

    primitive_id: str
    duration_s: float
    positions: Tuple[Vector3, ...]

    def __post_init__(self) -> None:
        if self.duration_s <= 0.0:
            raise ValueError("duration_s must be positive")
        if len(self.positions) < 2:
            raise ValueError("a primitive needs at least two samples")

    @property
    def start(self) -> Vector3:
        return self.positions[0]

    @property
    def terminal(self) -> Vector3:
        return self.positions[-1]


@dataclass(frozen=True)
class CertificateLibrary:
    """Worst-case collision-risk bounds indexed by primitive and clearance bin."""

    distance_bins_m: Tuple[float, ...]
    primitives: Tuple[MotionPrimitive, ...]
    risks: Tuple[Tuple[float, ...], ...]

    def __post_init__(self) -> None:
        if not self.distance_bins_m:
            raise ValueError("at least one distance bin is required")
        if tuple(sorted(self.distance_bins_m)) != self.distance_bins_m:
            raise ValueError("distance bins must be sorted")
        if any(distance <= 0.0 for distance in self.distance_bins_m):
            raise ValueError("distance bins must be positive")
        if len(self.risks) != len(self.primitives):
            raise ValueError("one risk row is required per primitive")
        if any(len(row) != len(self.distance_bins_m) for row in self.risks):
            raise ValueError("every risk row must cover every distance bin")


@dataclass(frozen=True)
class SelectionResult:
    """Online selection outcome; primitive_id is absent during fail-safe hold."""

    primitive_id: Optional[str]
    risk_bound: Optional[float]
    threshold: float
    distance_bin_m: float
    fallback: bool
    reason: str
