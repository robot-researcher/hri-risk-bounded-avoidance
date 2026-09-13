"""Bounded-cost online primitive selection with explicit fail-safe behavior."""

from bisect import bisect_right

from .models import CertificateLibrary, SelectionResult, Vector3


class PrimitiveSelector:
    def __init__(self, library: CertificateLibrary, candidate_limit: int = 16) -> None:
        if candidate_limit <= 0:
            raise ValueError("candidate_limit must be positive")
        self.library = library
        self.candidate_limit = candidate_limit
        self._sorted_by_bin = tuple(
            tuple(sorted((library.risks[index][bin_index], index) for index in range(len(library.primitives))))
            for bin_index in range(len(library.distance_bins_m))
        )

    def _conservative_bin_index(self, obstacle_distance_m: float) -> int:
        index = bisect_right(self.library.distance_bins_m, obstacle_distance_m) - 1
        return max(0, min(index, len(self.library.distance_bins_m) - 1))

    def select(
        self,
        *,
        obstacle_distance_m: float,
        threshold: float,
        goal_joint_angles: Vector3,
    ) -> SelectionResult:
        bin_index = self._conservative_bin_index(obstacle_distance_m)
        distance_bin = self.library.distance_bins_m[bin_index]
        ranked = self._sorted_by_bin[bin_index]
        feasible_count = bisect_right(ranked, (threshold, len(self.library.primitives)))
        if feasible_count == 0:
            return SelectionResult(
                primitive_id=None,
                risk_bound=None,
                threshold=threshold,
                distance_bin_m=distance_bin,
                fallback=True,
                reason="no certified primitive satisfies the active risk threshold",
            )

        candidates = ranked[: min(feasible_count, self.candidate_limit)]
        risk, primitive_index = min(
            candidates,
            key=lambda item: sum(
                (actual - target) ** 2
                for actual, target in zip(self.library.primitives[item[1]].terminal, goal_joint_angles)
            ),
        )
        primitive = self.library.primitives[primitive_index]
        return SelectionResult(
            primitive_id=primitive.primitive_id,
            risk_bound=risk,
            threshold=threshold,
            distance_bin_m=distance_bin,
            fallback=False,
            reason="selected lowest goal-cost primitive from bounded feasible set",
        )
