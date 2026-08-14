import json
from pathlib import Path

from shapely.geometry import LineString, box

from coverage_planner.models import CameraConfig, SemanticMap
from coverage_planner.planner import CoveragePlanner
from coverage_planner.reporting import export_plan


def semantic_map() -> SemanticMap:
    return SemanticMap.model_validate({"schema_version":"1.0","world_name":"tiny","coordinate_frame":"ENU","units":"meters",
      "search_area":{"kind":"rectangle","coords":[[0,0],[40,0],[40,30],[0,30]]},"nodes":[],
      "metadata":{"ground_truth_excluded":True,"source":"test"}})


def camera() -> CameraConfig:
    return CameraConfig.model_validate({"image_width_px":100,"image_height_px":100,
      "horizontal_fov_deg":90,"vertical_fov_deg":90,"pitch_deg":-90,"yaw_mode":"follow_path",
      "forward_overlap":.5,"side_overlap":.5})


def test_planner_exports_all_required_artifacts(tmp_path: Path) -> None:
    result = CoveragePlanner().plan(semantic_map=semantic_map(), search_geometry=box(0,0,40,30),
      camera=camera(),flight_altitude_m=10,start=(0,0,10),horizontal_clearance_m=1,
      vertical_clearance_m=2,scan_direction_deg=90)
    export_plan(result,tmp_path)
    expected={"patches.geojson","route.geojson","coverage_report.json",
              "visualization.png","flight_plan.json","flight_plan.yaml"}
    assert {path.name for path in tmp_path.iterdir()} == expected
    assert (tmp_path/"visualization.png").stat().st_size>10_000
    assert (result.planning_route[0].x, result.planning_route[0].y) == (
        result.planning_route[-1].x, result.planning_route[-1].y
    )
    assert not result.planning_route[-1].capture
    flight=json.loads((tmp_path/"flight_plan.json").read_text())
    assert flight["schema_version"] == "3.0"
    assert flight["video_detection"]["mode"] == "continuous_video_stream"
    assert flight["video_detection"]["analysis_rate_hz"] == 2
    assert all({"heading_deg", "speed_mps"} <= waypoint.keys()
               for waypoint in flight["waypoints"])
    assert all(segment["speed_mps"] > 0 for segment in flight["route_segments"])
    report=json.loads((tmp_path/"coverage_report.json").read_text())
    assert report["optimization_method"] == "layered_deterministic_heuristic"
    assert report["initial_candidate_metrics"]
    assert report["final_solution_metrics"]["coverage_ratio"] == report["coverage_ratio"]
    assert report["unreachable_candidate_point_count"] == len(
        report["unreachable_candidate_point_ids"])
    assert report["uncovered_patch_count"] == len(report["unreachable_patch_ids"])


def test_planner_is_deterministic() -> None:
    kwargs={"semantic_map":semantic_map(),"search_geometry":box(0,0,40,30),"camera":camera(),
      "flight_altitude_m":10,"start":(0,0,10),"scan_direction_deg":None}
    assert CoveragePlanner().plan(**kwargs)==CoveragePlanner().plan(**kwargs)  # type: ignore[arg-type]


def test_lawn_mower_returns_home_and_avoids_obstacles() -> None:
    result = CoveragePlanner().plan(
        semantic_map=semantic_map(), search_geometry=box(0, 0, 40, 30), camera=camera(),
        flight_altitude_m=10, start=(20, 15, 10), scan_pattern="lawn_mower")
    assert {item.pattern for item in result.strategy_comparison} == {"scanline_clipped"}
    assert result.scan_pattern == "scanline_clipped"
    assert (result.planning_route[0].x, result.planning_route[0].y) == (20, 15)
    assert (result.planning_route[-1].x, result.planning_route[-1].y) == (20, 15)
    points = {waypoint.id: waypoint for waypoint in result.continuous_flight.waypoints}
    for segment in result.continuous_flight.route_segments:
        start, end = points[segment.start_waypoint_id], points[segment.end_waypoint_id]
        assert LineString([(start.x, start.y), (end.x, end.y)]).relate(
            result.obstacles.geometry)[0] == "F"


def test_bcd_is_a_parallel_coverage_generator() -> None:
    result = CoveragePlanner().plan(
        semantic_map=semantic_map(), search_geometry=box(0, 0, 40, 30), camera=camera(),
        flight_altitude_m=10, start=(20, 15, 10), scan_pattern="bcd",
        scan_direction_deg=90)
    assert result.scan_pattern == "bcd"
    assert result.coverage_requirement_met
    assert result.continuous_flight.lanes


def test_low_altitude_coverage_uses_only_reachable_captures() -> None:
    map_with_building = SemanticMap.model_validate({"schema_version":"1.0","world_name":"blocked",
      "coordinate_frame":"ENU","units":"meters","search_area":{"kind":"rectangle","coords":[[0,0],[30,0],[30,20],[0,20]]},
      "nodes":[{"id":"building","properties":{"category":"building","type":"office","label":"building",
      "passability":"restricted","visibility":"public","elevation_min_m":0,"elevation_max_m":20},
      "shape":{"type":"rectangle","min_corner":[10,5],"max_corner":[20,15]}}],
      "metadata":{"ground_truth_excluded":True,"source":"test"}})
    result = CoveragePlanner().plan(semantic_map=map_with_building,search_geometry=box(0,0,30,20),
      camera=camera(),flight_altitude_m=10,start=(0,0,10),horizontal_clearance_m=3,
      vertical_clearance_m=2,scan_direction_deg=90)
    assert all(
        contributor.startswith("segment_")
        for patch in result.patches for contributor in patch.covered_by_waypoint_ids)
