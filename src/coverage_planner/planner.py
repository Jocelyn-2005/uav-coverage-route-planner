"""Pure-Python end-to-end coverage planner orchestration."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import pairwise
from math import hypot
from typing import Literal

from shapely import unary_union
from shapely.geometry import MultiPolygon, Point, Polygon
from shapely.geometry.base import BaseGeometry

from coverage_planner.camera import ground_footprint_dimensions
from coverage_planner.coverage import (
    build_continuous_flight_plan,
    evaluate_patch_coverage,
    generate_contour_capture_plan,
    optimize_scan_direction,
    prepare_lane_route,
    supplement_uncovered_patches,
)
from coverage_planner.coverage.scanlines import CapturePlan
from coverage_planner.geometry import build_effective_search_area
from coverage_planner.models import (
    CameraConfig,
    ContinuousFlightPlan,
    Patch,
    PatchGridConfig,
    SemanticMap,
    Waypoint,
)
from coverage_planner.models.search_area import EffectiveSearchArea, Polygonal
from coverage_planner.partition import generate_patches
from coverage_planner.routing import (
    FlightObstacles,
    route_reachable_waypoints,
    select_flight_obstacles,
)
from coverage_planner.visibility import visible_detection_ground


@dataclass(frozen=True, slots=True)
class StrategyMetrics:
    pattern: str
    coverage_ratio: float
    planning_point_count: int
    path_length_m: float
    unreachable_patch_count: int


@dataclass(frozen=True, slots=True)
class PatternCandidate:
    pattern: str
    capture_plan: CapturePlan
    planning_route: tuple[Waypoint, ...]
    skipped_point_ids: tuple[str, ...]
    metrics: StrategyMetrics


@dataclass(frozen=True, slots=True)
class PlanResult:
    semantic_map: SemanticMap
    effective_area: EffectiveSearchArea
    patches: tuple[Patch, ...]
    planning_route: tuple[Waypoint, ...]
    obstacles: FlightObstacles
    scan_direction_deg: float
    unreachable_patch_ids: tuple[str, ...]
    warnings: tuple[str, ...]
    continuous_flight: ContinuousFlightPlan
    visible_detection_geometry: Polygonal
    scan_pattern: str = "lawn_mower"
    strategy_comparison: tuple[StrategyMetrics, ...] = ()

    @property
    def path_length_m(self) -> float:
        return sum(hypot(b.x-a.x, b.y-a.y) for a, b in pairwise(self.planning_route))


class CoveragePlanner:
    def plan(
        self, *, semantic_map: SemanticMap, search_geometry: BaseGeometry,
        camera: CameraConfig, flight_altitude_m: float, start: tuple[float, float, float],
        horizontal_clearance_m: float = 3.0, vertical_clearance_m: float = 2.0,
        allow_overflight_above_buildings: bool = True, scan_direction_deg: float | None = None,
        patch_config: PatchGridConfig | None = None, ground_elevation_m: float = 0.0,
        minimum_coverage_ratio: float = 0.95,
        return_to_start: bool = True,
        scan_pattern: Literal["lawn_mower", "contour_outward", "auto"] = "lawn_mower",
        video_analysis_rate_hz: float = 2.0,
        control_point_spacing_m: float = 10.0,
        coverage_speed_mps: float = 5.0,
        connector_speed_mps: float = 4.0,
        obstacle_speed_mps: float = 2.5,
        return_speed_mps: float = 4.0,
    ) -> PlanResult:
        effective = build_effective_search_area(semantic_map, search_geometry)
        dimensions = ground_footprint_dimensions(camera, flight_altitude_m=flight_altitude_m,
                                                  ground_elevation_m=ground_elevation_m)
        patches = generate_patches(effective.geometry, config=patch_config, camera_dimensions=dimensions)
        obstacles = select_flight_obstacles(
            semantic_map, flight_altitude_m=flight_altitude_m,
            vertical_clearance_m=vertical_clearance_m,
            horizontal_clearance_m=horizontal_clearance_m,
            allow_overflight_above_buildings=allow_overflight_above_buildings)
        patterns = ("lawn_mower", "contour_outward") if scan_pattern == "auto" else (scan_pattern,)
        candidates = tuple(self._run_pattern(
            pattern=pattern, effective_geometry=effective.geometry, patches=patches,
            semantic_map=semantic_map,
            camera=camera, flight_altitude_m=flight_altitude_m,
            ground_elevation_m=ground_elevation_m, scan_direction_deg=scan_direction_deg,
            minimum_coverage_ratio=minimum_coverage_ratio, start=start,
            obstacle_geometry=obstacles.geometry, return_to_start=return_to_start,
        ) for pattern in patterns)
        best_coverage = max(item.metrics.coverage_ratio for item in candidates)
        competitive = tuple(
            item for item in candidates
            if item.metrics.coverage_ratio >= best_coverage - 0.01
        )
        chosen = min(competitive, key=lambda item: (
            item.metrics.unreachable_patch_count, item.metrics.path_length_m,
            -item.metrics.coverage_ratio, item.pattern))
        capture_plan = chosen.capture_plan
        planning_route = chosen.planning_route
        skipped_point_ids = chosen.skipped_point_ids
        continuous_flight, continuous_footprints = build_continuous_flight_plan(
            planning_route, camera=camera, flight_altitude_m=flight_altitude_m,
            ground_elevation_m=ground_elevation_m,
            video_analysis_rate_hz=video_analysis_rate_hz,
            control_point_spacing_m=control_point_spacing_m,
            coverage_speed_mps=coverage_speed_mps,
            connector_speed_mps=connector_speed_mps,
            obstacle_speed_mps=obstacle_speed_mps,
            return_speed_mps=return_speed_mps, capture_region=effective.geometry,
            semantic_map=semantic_map)
        evaluated = evaluate_patch_coverage(
            patches, continuous_footprints, minimum_coverage_ratio=minimum_coverage_ratio
        )
        warnings = ([f"{len(skipped_point_ids)} coverage points are unreachable at the fixed altitude"]
                    if skipped_point_ids else [])
        unreachable = tuple(p.id for p in evaluated if not p.covered)
        comparison = tuple(item.metrics for item in sorted(candidates, key=lambda item: item.pattern))
        visible_union = (
            unary_union(tuple(continuous_footprints.values()))
            if continuous_footprints else Polygon())
        visible_union = visible_union.intersection(effective.geometry)
        visible_detection_geometry = (
            visible_union if isinstance(visible_union, (Polygon, MultiPolygon)) else Polygon())
        return PlanResult(semantic_map, effective, evaluated, planning_route, obstacles,
                          capture_plan.scan_direction_deg, unreachable, tuple(warnings),
                          continuous_flight, visible_detection_geometry,
                          chosen.pattern, comparison)

    def _run_pattern(
        self, *, pattern: str, effective_geometry: Polygonal, patches: tuple[Patch, ...],
        semantic_map: SemanticMap,
        camera: CameraConfig, flight_altitude_m: float, ground_elevation_m: float,
        scan_direction_deg: float | None, minimum_coverage_ratio: float,
        start: tuple[float, float, float], obstacle_geometry: Polygonal,
        return_to_start: bool,
    ) -> PatternCandidate:
        from coverage_planner.coverage import generate_capture_plan

        if pattern == "contour_outward":
            capture_plan = generate_contour_capture_plan(
                effective_geometry, camera=camera, flight_altitude_m=flight_altitude_m,
                ground_elevation_m=ground_elevation_m, center_enu_m=(start[0], start[1]))
        elif pattern == "lawn_mower":
            if scan_direction_deg is None:
                capture_plan, _ = optimize_scan_direction(
                    effective_geometry, camera=camera, flight_altitude_m=flight_altitude_m,
                    ground_elevation_m=ground_elevation_m)
            else:
                capture_plan = generate_capture_plan(
                    effective_geometry, camera=camera, flight_altitude_m=flight_altitude_m,
                    ground_elevation_m=ground_elevation_m, scan_direction_deg=scan_direction_deg)
        else:
            raise ValueError(f"unsupported scan pattern: {pattern}")
        capture_plan, _ = supplement_uncovered_patches(
            capture_plan, patches, camera=camera, flight_altitude_m=flight_altitude_m,
            ground_elevation_m=ground_elevation_m, minimum_coverage_ratio=minimum_coverage_ratio)
        start_wp = Waypoint("wp_start", 0, "transit", *start, 0, -90, False)
        route_points, pre_skipped = prepare_lane_route(
            capture_plan, start_enu_m=(start[0], start[1]), obstacles=obstacle_geometry)
        planning_route, skipped = route_reachable_waypoints(
            start_wp, route_points, obstacle_geometry,
            return_to_start=return_to_start)
        skipped = (*pre_skipped, *skipped)
        footprints = {
            w.id: visible_detection_ground(
                camera=camera, center_enu_m=(w.x, w.y),
                flight_altitude_m=flight_altitude_m,
                ground_elevation_m=ground_elevation_m, yaw_deg=w.yaw_deg,
                semantic_map=semantic_map,
            ).intersection(effective_geometry)
            for w in capture_plan.capture_waypoints
            if w.camera_footprint_enu is not None
            and not obstacle_geometry.covers(Point(w.x, w.y))
        }
        evaluated = evaluate_patch_coverage(
            patches, footprints, minimum_coverage_ratio=minimum_coverage_ratio)
        effective_area = sum(p.area_m2 for p in patches)
        covered_area = sum(p.area_m2 * p.coverage_ratio for p in evaluated)
        metrics = StrategyMetrics(
            pattern=pattern, coverage_ratio=covered_area / effective_area if effective_area else 0,
            planning_point_count=len(planning_route),
            path_length_m=sum(hypot(b.x-a.x, b.y-a.y)
                              for a, b in pairwise(planning_route)),
            unreachable_patch_count=sum(not p.covered for p in evaluated),
        )
        return PatternCandidate(pattern, capture_plan, planning_route, skipped, metrics)
