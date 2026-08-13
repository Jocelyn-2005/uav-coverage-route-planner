"""Deterministic scan-direction selection and uncovered-patch supplementation."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, replace
from itertools import pairwise
from math import hypot

from shapely import affinity
from shapely.geometry import LineString, MultiLineString, Point

from coverage_planner.camera import ground_footprint_polygon
from coverage_planner.coverage.evaluation import evaluate_patch_coverage
from coverage_planner.coverage.scanlines import CapturePlan, generate_capture_plan
from coverage_planner.models.camera import CameraConfig
from coverage_planner.models.patch import Patch
from coverage_planner.models.search_area import Polygonal
from coverage_planner.models.waypoint import Waypoint
from coverage_planner.routing.visibility import RoutingError, VisibilityRouter


@dataclass(frozen=True, slots=True)
class DirectionScore:
    angle_deg: float
    path_length_m: float
    deadhead_distance_m: float
    turn_count: int
    segment_count: int
    waypoint_count: int

    @property
    def ranking(self) -> tuple[float, float, int, int, int, float]:
        return (self.path_length_m, self.deadhead_distance_m, self.turn_count,
                self.segment_count, self.waypoint_count, self.angle_deg)


def optimize_scan_direction(
    geometry: Polygonal, *, camera: CameraConfig, flight_altitude_m: float,
    ground_elevation_m: float, candidate_angles_deg: Sequence[float] = tuple(range(0, 180, 15)),
) -> tuple[CapturePlan, tuple[DirectionScore, ...]]:
    if not candidate_angles_deg:
        raise ValueError("candidate_angles_deg cannot be empty")
    candidates = []
    for angle in candidate_angles_deg:
        plan = generate_capture_plan(
            geometry, camera=camera, flight_altitude_m=flight_altitude_m,
            ground_elevation_m=ground_elevation_m, scan_direction_deg=angle,
        )
        score = _score(plan)
        candidates.append((score, plan))
    candidates.sort(key=lambda item: item[0].ranking)
    return candidates[0][1], tuple(item[0] for item in candidates)


def supplement_uncovered_patches(
    plan: CapturePlan, patches: Sequence[Patch], *, camera: CameraConfig,
    flight_altitude_m: float, ground_elevation_m: float,
    minimum_coverage_ratio: float = 0.95, maximum_passes: int = 2,
) -> tuple[CapturePlan, tuple[Patch, ...]]:
    waypoints = list(plan.capture_waypoints)
    evaluated: tuple[Patch, ...] = tuple(patches)
    for _ in range(maximum_passes + 1):
        footprints = {
            waypoint.id: waypoint.camera_footprint_enu for waypoint in waypoints
            if waypoint.camera_footprint_enu is not None
        }
        evaluated = evaluate_patch_coverage(
            patches, footprints, minimum_coverage_ratio=minimum_coverage_ratio
        )
        uncovered = [patch for patch in evaluated if not patch.covered]
        if not uncovered:
            break
        for patch in uncovered:
            point = patch.geometry.representative_point()
            waypoint_id = f"wp_{len(waypoints) + 1:04d}"
            yaw = plan.scan_direction_deg
            waypoint = Waypoint(
                id=waypoint_id, sequence=len(waypoints) + 1, kind="capture",
                x=point.x, y=point.y, z=flight_altitude_m, yaw_deg=yaw,
                camera_pitch_deg=camera.pitch_deg, capture=True,
                camera_footprint_enu=ground_footprint_polygon(
                    camera, center_enu_m=(point.x, point.y),
                    flight_altitude_m=flight_altitude_m,
                    ground_elevation_m=ground_elevation_m, yaw_deg=yaw,
                ),
            )
            insertion_index = _cheapest_insertion_index(waypoints, waypoint)
            waypoints.insert(insertion_index, waypoint)
        waypoints = [replace(waypoint, sequence=index, id=f"wp_{index:04d}")
                     for index, waypoint in enumerate(waypoints, start=1)]
    return replace(plan, capture_waypoints=tuple(waypoints)), evaluated


def prepare_lane_route(
    plan: CapturePlan, *, start_enu_m: tuple[float, float], obstacles: Polygonal,
) -> tuple[tuple[Waypoint, ...], tuple[str, ...]]:
    """Reduce dense capture samples to ordered lane endpoints before routing."""
    jobs: list[tuple[Waypoint, ...]] = []
    skipped: list[str] = []
    current_key: tuple[int | None, int | None] | None = None
    current: list[Waypoint] = []

    def finish_job() -> None:
        if not current:
            return
        if len(current) == 1:
            if obstacles.covers(Point(current[0].x, current[0].y)):
                skipped.append(current[0].id)
            else:
                jobs.append((current[0],))
            return
        source = LineString([(current[0].x, current[0].y), (current[-1].x, current[-1].y)])
        clipped = source.difference(obstacles)
        parts = list(clipped.geoms) if isinstance(clipped, MultiLineString) else [clipped]
        usable = [part for part in parts if isinstance(part, LineString) and part.length > 2e-6]
        if not usable:
            skipped.extend(wp.id for wp in current)
            return
        for part in usable:
            # Visibility routing treats the safety-buffer boundary as blocked.
            start = Point(part.coords[0])
            end = Point(part.coords[-1])
            if obstacles.covers(start):
                start = part.interpolate(1e-6)
            if obstacles.covers(end):
                end = part.interpolate(part.length - 1e-6)
            endpoints = []
            for template, point in ((current[0], start), (current[-1], end)):
                footprint = template.camera_footprint_enu
                if footprint is not None:
                    footprint = affinity.translate(
                        footprint, xoff=point.x - template.x, yoff=point.y - template.y)
                endpoints.append(replace(
                    template, x=float(point.x), y=float(point.y),
                    camera_footprint_enu=footprint))
            jobs.append(tuple(endpoints))

    for waypoint in plan.capture_waypoints:
        key = (waypoint.scan_line_index, waypoint.scan_segment_index)
        if key == (None, None):
            finish_job()
            current = []
            current_key = None
            if obstacles.covers(Point(waypoint.x, waypoint.y)):
                skipped.append(waypoint.id)
            else:
                jobs.append((waypoint,))
            continue
        if current and key != current_key:
            finish_job()
            current = []
        current_key = key
        current.append(waypoint)
    finish_job()

    ordered: list[Waypoint] = []
    position = start_enu_m
    router = VisibilityRouter(obstacles)
    while jobs:
        choices: list[tuple[float, int, bool]] = []
        for index, job in enumerate(jobs):
            orientations = (False, True) if len(job) > 1 else (False,)
            for reverse in orientations:
                entry = job[-1] if reverse else job[0]
                try:
                    path = router.shortest_path(position, (entry.x, entry.y))
                    cost = sum(hypot(b[0] - a[0], b[1] - a[1]) for a, b in pairwise(path))
                except RoutingError:
                    cost = float("inf")
                choices.append((cost, index, reverse))
        _, index, reverse = min(choices, key=lambda item: (item[0], item[1], item[2]))
        job = jobs.pop(index)
        selected = tuple(reversed(job)) if reverse else job
        ordered.extend(selected)
        position = (selected[-1].x, selected[-1].y)

    return tuple(
        replace(waypoint, id=f"wp_{index:04d}", sequence=index)
        for index, waypoint in enumerate(ordered, 1)
    ), tuple(skipped)


def _cheapest_insertion_index(waypoints: Sequence[Waypoint], candidate: Waypoint) -> int:
    if len(waypoints) < 2:
        return len(waypoints)
    return min(
        range(1, len(waypoints) + 1),
        key=lambda index: (_insertion_cost(waypoints, candidate, index), index),
    )


def _insertion_cost(waypoints: Sequence[Waypoint], candidate: Waypoint, index: int) -> float:
    before = waypoints[index - 1]
    added = hypot(candidate.x - before.x, candidate.y - before.y)
    if index == len(waypoints):
        return added
    after = waypoints[index]
    return (added + hypot(after.x - candidate.x, after.y - candidate.y)
            - hypot(after.x - before.x, after.y - before.y))


def _score(plan: CapturePlan) -> DirectionScore:
    points = plan.capture_waypoints
    path_length = sum(hypot(b.x - a.x, b.y - a.y) for a, b in pairwise(points))
    deadhead = 0.0
    for a, b in pairwise(points):
        if (a.scan_line_index, a.scan_segment_index) != (b.scan_line_index, b.scan_segment_index):
            deadhead += hypot(b.x - a.x, b.y - a.y)
    return DirectionScore(
        angle_deg=plan.scan_direction_deg, path_length_m=path_length,
        deadhead_distance_m=deadhead, turn_count=max(0, len(plan.scan_segments) - 1),
        segment_count=len(plan.scan_segments), waypoint_count=len(points),
    )
