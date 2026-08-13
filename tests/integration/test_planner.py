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
    expected={"waypoints.json","waypoints.csv","patches.geojson","route.geojson",
              "coverage_report.json","visualization.png","flight_plan.json","flight_plan.yaml"}
    assert {path.name for path in tmp_path.iterdir()} == expected
    mission=json.loads((tmp_path/"waypoints.json").read_text())
    assert mission["coordinate_frame"]=="ENU"
    assert mission["schema_version"]=="1.0"
    assert mission["waypoints"]
    assert all({"x","y","z","yaw_deg","capture"} <= waypoint.keys() for waypoint in mission["waypoints"])
    capture = next(waypoint for waypoint in mission["waypoints"] if waypoint["capture"])
    assert capture["camera_footprint_enu"]["type"] == "Polygon"
    assert (tmp_path/"visualization.png").stat().st_size>10_000
    assert (result.waypoints[0].x, result.waypoints[0].y) == (
        result.waypoints[-1].x, result.waypoints[-1].y
    )
    assert not result.waypoints[-1].capture
    flight=json.loads((tmp_path/"flight_plan.json").read_text())
    assert flight["schema_version"] == "2.0"
    assert flight["capture"]["mode"] == "continuous"
    assert flight["capture"]["frequency_hz"] == 2
    assert all({"heading_deg", "speed_mps"} <= waypoint.keys()
               for waypoint in flight["waypoints"])
    assert all(segment["speed_mps"] > 0 for segment in flight["route_segments"])


def test_planner_is_deterministic() -> None:
    kwargs={"semantic_map":semantic_map(),"search_geometry":box(0,0,40,30),"camera":camera(),
      "flight_altitude_m":10,"start":(0,0,10),"scan_direction_deg":None}
    assert CoveragePlanner().plan(**kwargs)==CoveragePlanner().plan(**kwargs)  # type: ignore[arg-type]


def test_auto_compares_both_scan_patterns_and_returns_home() -> None:
    result = CoveragePlanner().plan(
        semantic_map=semantic_map(), search_geometry=box(0, 0, 40, 30), camera=camera(),
        flight_altitude_m=10, start=(20, 15, 10), scan_pattern="auto")
    assert {item.pattern for item in result.strategy_comparison} == {
        "lawn_mower", "contour_outward"}
    assert result.scan_pattern in {"lawn_mower", "contour_outward"}
    assert (result.waypoints[0].x, result.waypoints[0].y) == (20, 15)
    assert (result.waypoints[-1].x, result.waypoints[-1].y) == (20, 15)
    best_coverage = max(item.coverage_ratio for item in result.strategy_comparison)
    assert next(item for item in result.strategy_comparison
                if item.pattern == result.scan_pattern).coverage_ratio >= best_coverage - 0.01
    assert result.continuous_flight is not None
    points = {waypoint.id: waypoint for waypoint in result.continuous_flight.waypoints}
    for segment in result.continuous_flight.route_segments:
        start, end = points[segment.start_waypoint_id], points[segment.end_waypoint_id]
        assert LineString([(start.x, start.y), (end.x, end.y)]).relate(
            result.obstacles.geometry)[0] == "F"


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
    assert result.continuous_flight is not None
    assert all(
        contributor.startswith("segment_")
        for patch in result.patches for contributor in patch.covered_by_waypoint_ids)
