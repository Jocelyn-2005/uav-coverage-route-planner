"""Canonical lane-routing representation shared by heuristic and exact solvers."""

from __future__ import annotations

from dataclasses import dataclass, replace
from itertools import pairwise
from math import hypot

from shapely import affinity
from shapely.geometry import LineString, MultiLineString, Point

from coverage_planner.coverage.scanlines import CapturePlan
from coverage_planner.models.search_area import Polygonal
from coverage_planner.models.waypoint import Waypoint


@dataclass(frozen=True, slots=True)
class LaneJob:
    """One mandatory service lane with one or two executable orientations."""

    id: str
    waypoints: tuple[Waypoint, ...]
    scan_line_index: int | None
    scan_segment_index: int | None

    @property
    def orientations(self) -> tuple[tuple[Waypoint, ...], ...]:
        if len(self.waypoints) < 2:
            return (self.waypoints,)
        return (self.waypoints, tuple(reversed(self.waypoints)))

    @property
    def service_length_m(self) -> float:
        return sum(hypot(b.x - a.x, b.y - a.y)
                   for a, b in pairwise(self.waypoints))


@dataclass(frozen=True, slots=True)
class LaneRoutingProblem:
    """Geometry-fixed combinatorial problem for lane ordering and orientation."""

    start_enu_m: tuple[float, float]
    jobs: tuple[LaneJob, ...]
    obstacles: Polygonal


@dataclass(frozen=True, slots=True)
class LaneRoutingSolution:
    """A solver result before obstacle connectors are expanded into waypoints."""

    method: str
    ordered_waypoints: tuple[Waypoint, ...]
    job_order: tuple[str, ...]
    orientation_indices: tuple[int, ...]
    skipped_point_ids: tuple[str, ...]
    transition_cost_m: float
    return_cost_m: float


def build_lane_routing_problem(
    plan: CapturePlan, *, start_enu_m: tuple[float, float], obstacles: Polygonal,
) -> tuple[LaneRoutingProblem, tuple[str, ...]]:
    """Clip dense scan samples into mandatory lane jobs without ordering them."""
    jobs: list[LaneJob] = []
    skipped: list[str] = []
    current_key: tuple[int | None, int | None] | None = None
    current: list[Waypoint] = []

    def finish_job() -> None:
        if not current:
            return
        key = (current[0].scan_line_index, current[0].scan_segment_index)
        if len(current) == 1:
            if obstacles.covers(Point(current[0].x, current[0].y)):
                skipped.append(current[0].id)
            else:
                jobs.append(LaneJob(_job_id(len(jobs)), (current[0],), *key))
            return
        source = LineString([(current[0].x, current[0].y), (current[-1].x, current[-1].y)])
        clipped = source.difference(obstacles)
        parts = list(clipped.geoms) if isinstance(clipped, MultiLineString) else [clipped]
        usable = [part for part in parts if isinstance(part, LineString) and part.length > 2e-6]
        if not usable:
            skipped.extend(waypoint.id for waypoint in current)
            return
        for part in usable:
            start = Point(part.coords[0])
            end = Point(part.coords[-1])
            if obstacles.covers(start):
                start = part.interpolate(1e-6)
            if obstacles.covers(end):
                end = part.interpolate(part.length - 1e-6)
            endpoints = tuple(
                _move_waypoint(template, point)
                for template, point in ((current[0], start), (current[-1], end))
            )
            jobs.append(LaneJob(_job_id(len(jobs)), endpoints, *key))

    for waypoint in plan.capture_waypoints:
        key = (waypoint.scan_line_index, waypoint.scan_segment_index)
        if key == (None, None):
            finish_job()
            current = []
            current_key = None
            if obstacles.covers(Point(waypoint.x, waypoint.y)):
                skipped.append(waypoint.id)
            else:
                jobs.append(LaneJob(_job_id(len(jobs)), (waypoint,), None, None))
            continue
        if current and key != current_key:
            finish_job()
            current = []
        current_key = key
        current.append(waypoint)
    finish_job()
    return LaneRoutingProblem(start_enu_m, tuple(jobs), obstacles), tuple(skipped)


def renumber_waypoints(waypoints: tuple[Waypoint, ...]) -> tuple[Waypoint, ...]:
    return tuple(
        replace(waypoint, id=f"wp_{index:04d}", sequence=index)
        for index, waypoint in enumerate(waypoints, 1)
    )


def _move_waypoint(template: Waypoint, point: Point) -> Waypoint:
    footprint = template.camera_footprint_enu
    if footprint is not None:
        footprint = affinity.translate(
            footprint, xoff=point.x - template.x, yoff=point.y - template.y)
    return replace(
        template, x=float(point.x), y=float(point.y), camera_footprint_enu=footprint)


def _job_id(index: int) -> str:
    return f"lane_job_{index + 1:04d}"
