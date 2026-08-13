"""Validated domain models."""

from coverage_planner.models.camera import CameraConfig
from coverage_planner.models.patch import Patch, PatchGridConfig
from coverage_planner.models.search_area import EffectiveSearchArea, SearchAreaMetrics
from coverage_planner.models.semantic_map import SemanticMap
from coverage_planner.models.waypoint import ScanSegment, Waypoint

__all__ = [
    "CameraConfig",
    "EffectiveSearchArea",
    "Patch",
    "PatchGridConfig",
    "ScanSegment",
    "SearchAreaMetrics",
    "SemanticMap",
    "Waypoint",
]
