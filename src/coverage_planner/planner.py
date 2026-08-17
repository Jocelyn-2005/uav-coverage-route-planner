"""Pure-Python end-to-end coverage planner orchestration."""

from __future__ import annotations

from dataclasses import dataclass, replace
from itertools import pairwise
from math import hypot
from typing import Literal

from shapely import unary_union
from shapely.geometry import LineString, MultiPolygon, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import nearest_points

from coverage_planner.camera import ground_footprint_dimensions, ground_footprint_polygon
from coverage_planner.coverage import (
    build_continuous_flight_plan,
    evaluate_patch_coverage,
    optimize_scan_direction,
)
from coverage_planner.coverage.generators import BCDGenerator, GlobalScanlineGenerator
from coverage_planner.coverage.generators.base import CoverageStructureGenerator
from coverage_planner.coverage.scanlines import CapturePlan
from coverage_planner.geometry import build_effective_search_area
from coverage_planner.io.semantic_map import building_safety_geometry
from coverage_planner.models import (
    CameraConfig,
    ContinuousFlightPlan,
    Patch,
    PatchGridConfig,
    SemanticMap,
    Waypoint,
)
from coverage_planner.models.search_area import EffectiveSearchArea, Polygonal
from coverage_planner.optimization import (
    OptimizedRoute,
    RouteOptimizationMethod,
    build_route_optimization_problem,
    optimize_route,
)
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
    route_solution: OptimizedRoute
    route_candidates: tuple[OptimizedRoute, ...]


@dataclass(frozen=True, slots=True)
class UnreachableGround:
    geometry: Polygonal
    area_m2: float
    patch_ids: tuple[str, ...]
    reason: str


@dataclass(frozen=True, slots=True)
class PlanResult:
    semantic_map: SemanticMap
    effective_area: EffectiveSearchArea
    patches: tuple[Patch, ...]
    planning_route: tuple[Waypoint, ...]
    obstacles: FlightObstacles
    scan_direction_deg: float
    unreachable_candidate_point_ids: tuple[str, ...]
    unreachable_patch_ids: tuple[str, ...]
    warnings: tuple[str, ...]
    continuous_flight: ContinuousFlightPlan
    visible_detection_geometry: Polygonal
    visibility_samples: tuple[tuple[str, Polygonal], ...]
    minimum_obstacle_clearance_m: float | None
    minimum_required_coverage_ratio: float
    coverage_requirement_met: bool
    route_optimization_method: str
    route_optimization_candidates: tuple[OptimizedRoute, ...]
    unreachable_ground: tuple[UnreachableGround, ...]
    scan_pattern: str = "global_scanline"

    @property
    def coverage_generation_method(self) -> str:
        """Canonical name for the geometry method; scan_pattern is legacy output."""
        return self.scan_pattern
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
        minimum_coverage_ratio: float = 0.9999,
        return_to_start: bool = True,
        coverage_generation_method: Literal["global_scanline", "bcd"] | None = None,
        scan_pattern: Literal["scanline_clipped", "bcd", "lawn_mower"] | None = None,
        video_analysis_rate_hz: float = 2.0,
        control_point_spacing_m: float = 10.0,
        coverage_speed_mps: float = 5.0,
        connector_speed_mps: float = 4.0,
        obstacle_speed_mps: float = 2.5,
        return_speed_mps: float = 4.0,
        route_optimization_method: RouteOptimizationMethod = "auto",
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
        legacy_method = (
            "global_scanline" if scan_pattern in {"lawn_mower", "scanline_clipped"}
            else scan_pattern)
        if (coverage_generation_method is not None and legacy_method is not None
                and coverage_generation_method != legacy_method):
            raise ValueError(
                "coverage_generation_method conflicts with legacy scan_pattern")
        canonical_pattern = coverage_generation_method or legacy_method or "global_scanline"
        patterns = (canonical_pattern,)
        candidates = tuple(self._run_pattern(
            pattern=pattern, effective_geometry=effective.geometry, patches=patches,
            semantic_map=semantic_map,
            camera=camera, flight_altitude_m=flight_altitude_m,
            ground_elevation_m=ground_elevation_m, scan_direction_deg=scan_direction_deg,
            minimum_coverage_ratio=minimum_coverage_ratio, start=start,
            obstacle_geometry=obstacles.geometry, return_to_start=return_to_start,
            route_optimization_method=route_optimization_method,
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
        route_solution = chosen.route_solution
        route_candidates = chosen.route_candidates
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
        safe_observation_geometry = effective.geometry.difference(
            effective.building_exclusion_geometry.buffer(horizontal_clearance_m))
        attempted_supplement_points: set[tuple[str, float, float]] = set()
        for _ in range(6):
            unresolved = [patch for patch in evaluated if not patch.covered]
            if not unresolved:
                break
            supplemental_waypoints = list(capture_plan.capture_waypoints)
            covered_geometry = (
                unary_union(tuple(continuous_footprints.values()))
                if continuous_footprints else Polygon())
            added = False
            for patch in unresolved:
                uncovered_geometry = patch.geometry.difference(covered_geometry)
                if uncovered_geometry.is_empty:
                    continue
                point = self._coverage_completion_point(
                    uncovered_geometry,
                    safe_observation_geometry=safe_observation_geometry,
                    current_route=planning_route,
                    camera=camera,
                    flight_altitude_m=flight_altitude_m,
                    ground_elevation_m=ground_elevation_m,
                    yaw_deg=capture_plan.scan_direction_deg,
                    semantic_map=semantic_map,
                    effective_geometry=effective.geometry,
                )
                if point is None:
                    continue
                attempt_key = (patch.id, round(point.x, 6), round(point.y, 6))
                if attempt_key in attempted_supplement_points:
                    continue
                attempted_supplement_points.add(attempt_key)
                sample_id = f"wp_{len(supplemental_waypoints) + 1:04d}"
                supplemental_waypoints.append(Waypoint(
                    id=sample_id, sequence=len(supplemental_waypoints) + 1,
                    kind="capture", x=point.x, y=point.y, z=flight_altitude_m,
                    yaw_deg=capture_plan.scan_direction_deg,
                    camera_pitch_deg=camera.pitch_deg, capture=True,
                    camera_footprint_enu=ground_footprint_polygon(
                        camera, center_enu_m=(point.x, point.y),
                        flight_altitude_m=flight_altitude_m,
                        ground_elevation_m=ground_elevation_m,
                        yaw_deg=capture_plan.scan_direction_deg)))
                added = True
            if not added:
                break
            capture_plan = replace(
                capture_plan, capture_waypoints=tuple(supplemental_waypoints))
            route_points, post_skipped, route_solution, route_candidates = (
                self._optimize_coverage_route(
                    capture_plan, start_enu_m=(start[0], start[1]),
                    obstacles=obstacles.geometry,
                    method=route_optimization_method))
            planning_route, post_route_skipped = route_reachable_waypoints(
                Waypoint("wp_start", 0, "transit", *start, 0, -90, False),
                route_points, obstacles.geometry, return_to_start=return_to_start)
            skipped_point_ids = tuple(dict.fromkeys(
                (*skipped_point_ids, *post_skipped, *post_route_skipped)))
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
                patches, continuous_footprints,
                minimum_coverage_ratio=minimum_coverage_ratio)
        (capture_plan, planning_route, skipped_point_ids, route_solution,
         route_candidates, continuous_flight, continuous_footprints, evaluated) = (
            self._prune_redundant_completion_points(
                capture_plan,
                planning_route=planning_route,
                skipped_point_ids=skipped_point_ids,
                route_solution=route_solution,
                route_candidates=route_candidates,
                continuous_flight=continuous_flight,
                continuous_footprints=continuous_footprints,
                evaluated=evaluated,
                patches=patches,
                camera=camera,
                flight_altitude_m=flight_altitude_m,
                ground_elevation_m=ground_elevation_m,
                semantic_map=semantic_map,
                effective_geometry=effective.geometry,
                obstacle_geometry=obstacles.geometry,
                start=start,
                return_to_start=return_to_start,
                route_optimization_method=route_optimization_method,
                video_analysis_rate_hz=video_analysis_rate_hz,
                control_point_spacing_m=control_point_spacing_m,
                coverage_speed_mps=coverage_speed_mps,
                connector_speed_mps=connector_speed_mps,
                obstacle_speed_mps=obstacle_speed_mps,
                return_speed_mps=return_speed_mps,
                minimum_coverage_ratio=minimum_coverage_ratio,
            ))
        unreachable = tuple(p.id for p in evaluated if not p.covered)
        unreachable_ground = self._unreachable_ground(
            evaluated, continuous_footprints, obstacles.geometry)
        effective_area_m2 = effective.geometry.area
        covered_area_m2 = sum(p.area_m2 * p.coverage_ratio for p in evaluated)
        achieved_coverage_ratio = (
            covered_area_m2 / effective_area_m2 if effective_area_m2 else 0.0)
        coverage_requirement_met = (
            not unreachable
            and achieved_coverage_ratio >= minimum_coverage_ratio)
        warnings = []
        if skipped_point_ids:
            if coverage_requirement_met:
                warnings.append(
                    f"{len(skipped_point_ids)} initial candidate coverage points were "
                    "unreachable at the fixed altitude, but final ground coverage met "
                    "the requirement; these points do not represent uncovered ground")
            else:
                warnings.append(
                    f"{len(skipped_point_ids)} initial candidate coverage points were "
                    "unreachable at the fixed altitude; refer to unreachable_patch_ids "
                    "for ground that remains below the coverage requirement")
        if not coverage_requirement_met:
            unresolved_area_m2 = sum(
                patch.area_m2 * (1.0 - patch.coverage_ratio)
                for patch in evaluated if not patch.covered)
            warnings.append(
                f"{len(unreachable)} search patches remain below the required "
                f"{minimum_coverage_ratio:.4f} coverage, with approximately "
                f"{unresolved_area_m2:.2f} m^2 unresolved; mission is not ready "
                "for execution")
        comparison = tuple(item.metrics for item in sorted(candidates, key=lambda item: item.pattern))
        visible_union = (
            unary_union(tuple(continuous_footprints.values()))
            if continuous_footprints else Polygon())
        visible_union = visible_union.intersection(effective.geometry)
        visible_detection_geometry = (
            visible_union if isinstance(visible_union, (Polygon, MultiPolygon)) else Polygon())
        route_coordinates = [(waypoint.x, waypoint.y) for waypoint in planning_route]
        route_geometry: BaseGeometry = (
            LineString(route_coordinates) if len(route_coordinates) > 1
            else Point(route_coordinates[0]))
        blocked_nodes = tuple(node for node in semantic_map.building_nodes
                              if node.id in obstacles.building_ids)
        minimum_clearance = (
            min(route_geometry.distance(building_safety_geometry(semantic_map, node))
                for node in blocked_nodes)
            if blocked_nodes else None)
        return PlanResult(semantic_map, effective, evaluated, planning_route, obstacles,
                          capture_plan.scan_direction_deg, skipped_point_ids, unreachable,
                          tuple(warnings),
                          continuous_flight, visible_detection_geometry,
                          tuple(continuous_footprints.items()),
                          minimum_clearance,
                          minimum_coverage_ratio, coverage_requirement_met,
                          route_solution.method, route_candidates, unreachable_ground,
                          chosen.pattern, comparison)

    def _run_pattern(
        self, *, pattern: str, effective_geometry: Polygonal, patches: tuple[Patch, ...],
        semantic_map: SemanticMap,
        camera: CameraConfig, flight_altitude_m: float, ground_elevation_m: float,
        scan_direction_deg: float | None, minimum_coverage_ratio: float,
        start: tuple[float, float, float], obstacle_geometry: Polygonal,
        return_to_start: bool,
        route_optimization_method: RouteOptimizationMethod,
    ) -> PatternCandidate:
        generator: CoverageStructureGenerator
        if pattern == "global_scanline":
            generator = GlobalScanlineGenerator()
        elif pattern == "bcd":
            generator = BCDGenerator()
        else:
            raise ValueError(f"unsupported coverage generator: {pattern}")
        if scan_direction_deg is None:
            capture_plan, _ = optimize_scan_direction(
                effective_geometry, camera=camera, flight_altitude_m=flight_altitude_m,
                ground_elevation_m=ground_elevation_m, generator=generator)
        else:
            capture_plan = generator.generate(
                effective_geometry, camera=camera, flight_altitude_m=flight_altitude_m,
                ground_elevation_m=ground_elevation_m, scan_direction_deg=scan_direction_deg)
        if pattern == "bcd":
            capture_plan = self._prune_redundant_coverage_lanes(
                capture_plan, patches=patches, camera=camera,
                flight_altitude_m=flight_altitude_m,
                ground_elevation_m=ground_elevation_m,
                semantic_map=semantic_map,
                effective_geometry=effective_geometry,
                minimum_coverage_ratio=minimum_coverage_ratio)
        start_wp = Waypoint("wp_start", 0, "transit", *start, 0, -90, False)
        route_points, pre_skipped, route_solution, route_candidates = (
            self._optimize_coverage_route(
                capture_plan, start_enu_m=(start[0], start[1]),
                obstacles=obstacle_geometry, method=route_optimization_method))
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
        return PatternCandidate(
            pattern, capture_plan, planning_route, skipped, metrics,
            route_solution, route_candidates)

    @staticmethod
    def _optimize_coverage_route(
        capture_plan: CapturePlan,
        *,
        start_enu_m: tuple[float, float],
        obstacles: Polygonal,
        method: RouteOptimizationMethod,
    ) -> tuple[tuple[Waypoint, ...], tuple[str, ...], OptimizedRoute, tuple[OptimizedRoute, ...]]:
        problem, skipped = build_route_optimization_problem(
            capture_plan, start_enu_m=start_enu_m, obstacles=obstacles)
        solution, candidates = optimize_route(problem, method=method)
        return (
            solution.ordered_waypoints,
            (*skipped, *solution.skipped_point_ids),
            solution,
            candidates,
        )

    @staticmethod
    def _prune_redundant_coverage_lanes(
        capture_plan: CapturePlan,
        *,
        patches: tuple[Patch, ...],
        camera: CameraConfig,
        flight_altitude_m: float,
        ground_elevation_m: float,
        semantic_map: SemanticMap,
        effective_geometry: Polygonal,
        minimum_coverage_ratio: float,
    ) -> CapturePlan:
        waypoints = {waypoint.id: waypoint for waypoint in capture_plan.capture_waypoints}
        footprints = {
            waypoint.id: visible_detection_ground(
                camera=camera, center_enu_m=(waypoint.x, waypoint.y),
                flight_altitude_m=flight_altitude_m,
                ground_elevation_m=ground_elevation_m, yaw_deg=waypoint.yaw_deg,
                semantic_map=semantic_map).intersection(effective_geometry)
            for waypoint in capture_plan.capture_waypoints
        }
        retained = list(capture_plan.scan_segments)
        for segment in sorted(
                capture_plan.scan_segments,
                key=lambda item: (len(item.capture_waypoint_ids), item.scan_line_index,
                                  item.segment_index)):
            if len(segment.capture_waypoint_ids) > 1:
                continue
            candidate_footprints = {
                identifier: geometry for identifier, geometry in footprints.items()
                if identifier not in segment.capture_waypoint_ids}
            evaluated = evaluate_patch_coverage(
                patches, candidate_footprints,
                minimum_coverage_ratio=minimum_coverage_ratio)
            if not evaluated or any(not patch.covered for patch in evaluated):
                continue
            footprints = candidate_footprints
            retained.remove(segment)
            for identifier in segment.capture_waypoint_ids:
                waypoints.pop(identifier, None)
        ordered = tuple(
            replace(waypoint, id=f"wp_{index:04d}", sequence=index)
            for index, waypoint in enumerate(waypoints.values(), 1))
        id_by_position = {
            (waypoint.scan_line_index, waypoint.scan_segment_index, waypoint.x, waypoint.y): waypoint.id
            for waypoint in ordered}
        segments = tuple(replace(
            segment,
            capture_waypoint_ids=tuple(
                id_by_position[(
                    waypoints[identifier].scan_line_index,
                    waypoints[identifier].scan_segment_index,
                    waypoints[identifier].x,
                    waypoints[identifier].y,
                )]
                for identifier in segment.capture_waypoint_ids),
        ) for segment in retained)
        return replace(capture_plan, scan_segments=segments, capture_waypoints=ordered)

    @staticmethod
    def _unreachable_ground(
        patches: tuple[Patch, ...],
        footprints: dict[str, Polygonal],
        obstacles: Polygonal,
    ) -> tuple[UnreachableGround, ...]:
        covered = unary_union(tuple(footprints.values())) if footprints else Polygon()
        output = []
        for patch in patches:
            if patch.covered:
                continue
            geometry = patch.geometry.difference(covered)
            if geometry.is_empty:
                continue
            polygonal = geometry if isinstance(geometry, (Polygon, MultiPolygon)) else Polygon()
            reason = (
                "flight_obstacle_conflict" if obstacles.intersects(polygonal)
                else "camera_visibility_or_geometry_limit")
            output.append(UnreachableGround(
                polygonal, polygonal.area, (patch.id,), reason))
        return tuple(output)

    @staticmethod
    def _coverage_completion_point(
        uncovered_geometry: BaseGeometry,
        *,
        safe_observation_geometry: BaseGeometry,
        current_route: tuple[Waypoint, ...],
        camera: CameraConfig,
        flight_altitude_m: float,
        ground_elevation_m: float,
        yaw_deg: float,
        semantic_map: SemanticMap,
        effective_geometry: Polygonal,
    ) -> Point | None:
        if safe_observation_geometry.is_empty:
            return None
        target = uncovered_geometry.representative_point()
        nearest = nearest_points(target, safe_observation_geometry)[1]
        candidates = [nearest]
        dimensions = ground_footprint_dimensions(
            camera, flight_altitude_m=flight_altitude_m,
            ground_elevation_m=ground_elevation_m)
        radius = min(dimensions.width_m, dimensions.length_m) / 2.0
        for dx, dy in ((radius, 0.0), (-radius, 0.0), (0.0, radius), (0.0, -radius)):
            sample = Point(target.x + dx, target.y + dy)
            candidates.append(nearest_points(sample, safe_observation_geometry)[1])

        def score(point: Point) -> tuple[float, float]:
            visible = visible_detection_ground(
                camera=camera, center_enu_m=(point.x, point.y),
                flight_altitude_m=flight_altitude_m,
                ground_elevation_m=ground_elevation_m, yaw_deg=yaw_deg,
                semantic_map=semantic_map).intersection(effective_geometry)
            gain = visible.intersection(uncovered_geometry).area
            connection = min(
                (hypot(point.x - waypoint.x, point.y - waypoint.y)
                 for waypoint in current_route), default=0.0)
            return gain, -connection

        usable = [point for point in candidates
                  if safe_observation_geometry.covers(point)]
        if not usable:
            return None
        selected = max(usable, key=score)
        return selected if score(selected)[0] > 1e-6 else None

    def _prune_redundant_completion_points(
        self,
        capture_plan: CapturePlan,
        *,
        planning_route: tuple[Waypoint, ...],
        skipped_point_ids: tuple[str, ...],
        route_solution: OptimizedRoute,
        route_candidates: tuple[OptimizedRoute, ...],
        continuous_flight: ContinuousFlightPlan,
        continuous_footprints: dict[str, Polygonal],
        evaluated: tuple[Patch, ...],
        patches: tuple[Patch, ...],
        camera: CameraConfig,
        flight_altitude_m: float,
        ground_elevation_m: float,
        semantic_map: SemanticMap,
        effective_geometry: Polygonal,
        obstacle_geometry: Polygonal,
        start: tuple[float, float, float],
        return_to_start: bool,
        route_optimization_method: RouteOptimizationMethod,
        video_analysis_rate_hz: float,
        control_point_spacing_m: float,
        coverage_speed_mps: float,
        connector_speed_mps: float,
        obstacle_speed_mps: float,
        return_speed_mps: float,
        minimum_coverage_ratio: float,
    ) -> tuple[
        CapturePlan, tuple[Waypoint, ...], tuple[str, ...], OptimizedRoute,
        tuple[OptimizedRoute, ...], ContinuousFlightPlan, dict[str, Polygonal], tuple[Patch, ...],
    ]:
        completion_ids = [
            waypoint.id for waypoint in capture_plan.capture_waypoints
            if waypoint.scan_line_index is None and waypoint.scan_segment_index is None]
        for identifier in completion_ids:
            candidate_plan = replace(
                capture_plan,
                capture_waypoints=tuple(
                    waypoint for waypoint in capture_plan.capture_waypoints
                    if waypoint.id != identifier))
            route_points, candidate_skipped, candidate_solution, candidate_routes = (
                self._optimize_coverage_route(
                    candidate_plan, start_enu_m=(start[0], start[1]),
                    obstacles=obstacle_geometry, method=route_optimization_method))
            candidate_route, route_skipped = route_reachable_waypoints(
                Waypoint("wp_start", 0, "transit", *start, 0, -90, False),
                route_points, obstacle_geometry, return_to_start=return_to_start)
            candidate_flight, candidate_footprints = build_continuous_flight_plan(
                candidate_route, camera=camera, flight_altitude_m=flight_altitude_m,
                ground_elevation_m=ground_elevation_m,
                video_analysis_rate_hz=video_analysis_rate_hz,
                control_point_spacing_m=control_point_spacing_m,
                coverage_speed_mps=coverage_speed_mps,
                connector_speed_mps=connector_speed_mps,
                obstacle_speed_mps=obstacle_speed_mps,
                return_speed_mps=return_speed_mps, capture_region=effective_geometry,
                semantic_map=semantic_map)
            candidate_evaluated = evaluate_patch_coverage(
                patches, candidate_footprints,
                minimum_coverage_ratio=minimum_coverage_ratio)
            if any(not patch.covered for patch in candidate_evaluated):
                continue
            capture_plan = candidate_plan
            planning_route = candidate_route
            skipped_point_ids = tuple(dict.fromkeys(
                (*skipped_point_ids, *candidate_skipped, *route_skipped)))
            route_solution = candidate_solution
            route_candidates = candidate_routes
            continuous_flight = candidate_flight
            continuous_footprints = candidate_footprints
            evaluated = candidate_evaluated
        return (
            capture_plan, planning_route, skipped_point_ids, route_solution,
            route_candidates, continuous_flight, continuous_footprints, evaluated)
