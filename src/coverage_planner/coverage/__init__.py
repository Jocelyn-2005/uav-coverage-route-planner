"""Coverage generation and evaluation."""

from coverage_planner.coverage.contours import generate_contour_capture_plan
from coverage_planner.coverage.evaluation import (
    CoverageEvaluationError,
    evaluate_patch_coverage,
)
from coverage_planner.coverage.optimization import (
    DirectionScore,
    optimize_scan_direction,
    supplement_uncovered_patches,
)
from coverage_planner.coverage.scanlines import (
    CapturePlan,
    ScanlinePlanningError,
    generate_capture_plan,
)

__all__ = [
    "CapturePlan",
    "CoverageEvaluationError",
    "DirectionScore",
    "ScanlinePlanningError",
    "evaluate_patch_coverage",
    "generate_capture_plan",
    "generate_contour_capture_plan",
    "optimize_scan_direction",
    "supplement_uncovered_patches",
]
