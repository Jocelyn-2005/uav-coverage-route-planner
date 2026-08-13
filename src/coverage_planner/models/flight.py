"""Continuous-flight mission models for a lower-level vehicle adapter."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

SegmentKind = Literal["coverage_lane", "connector", "obstacle_avoidance", "return_home"]


@dataclass(frozen=True, slots=True)
class FlightWaypoint:
    """A route control point; image capture is continuous between these points."""

    id: str
    sequence: int
    x: float
    y: float
    z: float
    heading_deg: float
    speed_mps: float


@dataclass(frozen=True, slots=True)
class RouteSegment:
    """One constant-command straight segment of the planned flight route."""

    id: str
    sequence: int
    kind: SegmentKind
    start_waypoint_id: str
    end_waypoint_id: str
    heading_deg: float
    speed_mps: float
    length_m: float
    capture_enabled: bool
    source_scan_line_index: int | None = None
    source_scan_segment_index: int | None = None


@dataclass(frozen=True, slots=True)
class CoverageLane:
    """A maximal consecutive run of parallel ground-coverage segments."""

    id: str
    sequence: int
    heading_deg: float
    speed_mps: float
    route_segment_ids: tuple[str, ...]
    length_m: float


@dataclass(frozen=True, slots=True)
class ContinuousFlightPlan:
    """Continuous capture route and its explicit flight commands."""

    capture_frequency_hz: float
    control_point_spacing_m: float
    lane_overlap: float
    forward_overlap: float
    waypoints: tuple[FlightWaypoint, ...]
    route_segments: tuple[RouteSegment, ...]
    lanes: tuple[CoverageLane, ...]
    sampled_footprint_count: int
