"""Low-compute coverage planner for resource-constrained onboard computers."""

from __future__ import annotations

from itertools import pairwise
from math import hypot

from shapely import unary_union
from shapely.geometry import LineString, MultiPolygon, Point, Polygon
from shapely.geometry.base import BaseGeometry

from coverage_planner.camera import ground_footprint_dimensions
from coverage_planner.coverage import build_continuous_flight_plan, evaluate_patch_coverage
from coverage_planner.coverage.generators import GlobalScanlineGenerator
from coverage_planner.geometry import build_effective_search_area
from coverage_planner.io.semantic_map import building_safety_geometry
from coverage_planner.models import CameraConfig, PatchGridConfig, SemanticMap, Waypoint
from coverage_planner.optimization import build_route_optimization_problem, optimize_route
from coverage_planner.partition import generate_patches
from coverage_planner.planner import PlanResult, StrategyMetrics, UnreachableGround
from coverage_planner.routing import route_reachable_waypoints, select_flight_obstacles


class LightweightCoveragePlanner:
    """Generate one scanline plan, route it greedily, and verify ground coverage."""

    def plan(
        self, *, semantic_map: SemanticMap, search_geometry: BaseGeometry,
        camera: CameraConfig, flight_altitude_m: float,
        start: tuple[float, float, float], horizontal_clearance_m: float = 3.0,
        vertical_clearance_m: float = 2.0,
        allow_overflight_above_buildings: bool = True,
        scan_direction_deg: float | None = None,
        patch_config: PatchGridConfig | None = None,
        ground_elevation_m: float = 0.0,
        minimum_coverage_ratio: float = 0.9999,
        return_to_start: bool = True,
        video_analysis_rate_hz: float = 1.0,
        control_point_spacing_m: float = 10.0,
        coverage_speed_mps: float = 5.0,
        connector_speed_mps: float = 4.0,
        obstacle_speed_mps: float = 2.5,
        return_speed_mps: float = 4.0,
        **_ignored: object,
    ) -> PlanResult:
        effective = build_effective_search_area(semantic_map, search_geometry)
        dimensions = ground_footprint_dimensions(
            camera, flight_altitude_m=flight_altitude_m,
            ground_elevation_m=ground_elevation_m)
        patches = generate_patches(
            effective.geometry, config=patch_config, camera_dimensions=dimensions)
        obstacles = select_flight_obstacles(
            semantic_map, flight_altitude_m=flight_altitude_m,
            vertical_clearance_m=vertical_clearance_m,
            horizontal_clearance_m=horizontal_clearance_m,
            allow_overflight_above_buildings=allow_overflight_above_buildings)
        generator = GlobalScanlineGenerator()
        directions = (scan_direction_deg,) if scan_direction_deg is not None else (0.0, 90.0)
        plans = tuple(generator.generate(
            effective.geometry, camera=camera, flight_altitude_m=flight_altitude_m,
            ground_elevation_m=ground_elevation_m, scan_direction_deg=direction,
        ) for direction in directions)
        capture_plan = min(plans, key=lambda plan: (
            -unary_union(tuple(
                waypoint.camera_footprint_enu
                for waypoint in plan.capture_waypoints
                if waypoint.camera_footprint_enu is not None
            )).intersection(effective.geometry).area,
            len(plan.scan_segments),
            sum(hypot(b.x - a.x, b.y - a.y)
                for a, b in pairwise(plan.capture_waypoints)),
            plan.scan_direction_deg,
        ))
        problem, pre_skipped = build_route_optimization_problem(
            capture_plan, start_enu_m=(start[0], start[1]), obstacles=obstacles.geometry)
        route_solution, route_candidates = optimize_route(problem, method="greedy")
        start_waypoint = Waypoint("wp_start", 0, "transit", *start, 0, -90, False)
        planning_route, route_skipped = route_reachable_waypoints(
            start_waypoint, route_solution.ordered_waypoints, obstacles.geometry,
            return_to_start=return_to_start)
        continuous_flight, footprints = build_continuous_flight_plan(
            planning_route, camera=camera, flight_altitude_m=flight_altitude_m,
            ground_elevation_m=ground_elevation_m,
            video_analysis_rate_hz=video_analysis_rate_hz,
            control_point_spacing_m=control_point_spacing_m,
            coverage_speed_mps=coverage_speed_mps,
            connector_speed_mps=connector_speed_mps,
            obstacle_speed_mps=obstacle_speed_mps,
            return_speed_mps=return_speed_mps,
            capture_region=effective.geometry, semantic_map=None,
            wall_occlusion=False)
        evaluated = evaluate_patch_coverage(
            patches, footprints, minimum_coverage_ratio=minimum_coverage_ratio)
        unreachable = tuple(patch.id for patch in evaluated if not patch.covered)
        visible_geometry = unary_union(tuple(footprints.values())).intersection(
            effective.geometry)
        visible = (
            visible_geometry
            if isinstance(visible_geometry, (Polygon, MultiPolygon)) else Polygon())
        uncovered_ground = []
        for patch in evaluated:
            if patch.covered:
                continue
            geometry = patch.geometry.difference(visible)
            polygonal = geometry if isinstance(geometry, (Polygon, MultiPolygon)) else Polygon()
            uncovered_ground.append(UnreachableGround(
                polygonal, polygonal.area, (patch.id,), "geometric_coverage_limit"))
        warnings = (() if not unreachable else (
            f"{len(unreachable)} ground patches did not meet the coverage requirement",))
        route_coordinates = [(point.x, point.y) for point in planning_route]
        route_geometry = (
            LineString(route_coordinates) if len(route_coordinates) > 1 else Point(start[:2]))
        blocked_nodes = tuple(
            node for node in semantic_map.building_nodes
            if node.id in obstacles.building_ids)
        clearance = min(
            (route_geometry.distance(building_safety_geometry(semantic_map, node))
             for node in blocked_nodes), default=None)
        covered_area = sum(patch.area_m2 * patch.coverage_ratio for patch in evaluated)
        effective_area = effective.geometry.area
        metrics = StrategyMetrics(
            "global_scanline",
            covered_area / effective_area if effective_area else 0.0,
            len(planning_route),
            sum(hypot(right.x - left.x, right.y - left.y)
                for left, right in pairwise(planning_route)),
            len(unreachable),
        )
        skipped = tuple(dict.fromkeys((*pre_skipped, *route_skipped)))
        return PlanResult(
            semantic_map, effective, evaluated, planning_route, obstacles,
            capture_plan.scan_direction_deg, skipped, unreachable, warnings,
            continuous_flight, visible, tuple(footprints.items()), clearance,
            minimum_coverage_ratio, not unreachable,
            "greedy_obstacle_distance", route_candidates, tuple(uncovered_ground),
            "global_scanline", (metrics,),
        )
