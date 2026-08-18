import pytest
from pydantic import ValidationError

from coverage_planner.web import DualPlanRequest, PlanRequest, map_data


def test_map_api_payload_uses_real_yungu_data() -> None:
    payload = map_data()
    assert payload["world_name"] == "yungu2030_local_origin"
    assert len(payload["buildings"]) == 25
    assert payload["search_area"]["type"] == "Polygon"
    assert payload["background"]["url"] == "/api/background"
    assert len(payload["background"]["bounds"]) == 4


def test_web_request_rejects_invalid_camera() -> None:
    with pytest.raises(ValidationError, match="oblique camera projection not implemented"):
        PlanRequest.model_validate({
            "search_geometry":{"type":"Polygon","coordinates":[[[0,0],[1,0],[1,1],[0,1],[0,0]]]},
            "flight_altitude_m":30,"horizontal_clearance_m":3,
            "vertical_clearance_m":2,"camera":{"image_width_px":10,"image_height_px":10,
            "horizontal_fov_deg":60,"vertical_fov_deg":45,"pitch_deg":-45,
            "yaw_mode":"follow_path","forward_overlap":.3,"side_overlap":.3}})


def test_web_request_accepts_only_lightweight_generator() -> None:
    base = {
        "search_geometry": {
            "type": "Polygon",
            "coordinates": [[[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]]],
        },
        "flight_altitude_m": 30,
        "horizontal_clearance_m": 3,
        "vertical_clearance_m": 2,
        "camera": {
            "image_width_px": 10, "image_height_px": 10,
            "horizontal_fov_deg": 60, "vertical_fov_deg": 45,
            "pitch_deg": -90, "yaw_mode": "follow_path",
            "forward_overlap": .3, "side_overlap": .3,
        },
    }
    assert PlanRequest.model_validate({
        **base, "coverage_generation_method": "global_scanline"})
    with pytest.raises(ValidationError):
        PlanRequest.model_validate({**base, "coverage_generation_method": "bcd"})
    assert PlanRequest.model_validate({**base, "route_optimization_method": "auto"})
    with pytest.raises(ValidationError):
        PlanRequest.model_validate({**base, "route_optimization_method": "two_opt"})
    with pytest.raises(ValidationError):
        PlanRequest.model_validate({**base, "scan_direction_deg": 45})


def test_dual_web_request_requires_two_lightweight_assignments() -> None:
    base = {
        "flight_altitude_m": 30, "horizontal_clearance_m": 3,
        "vertical_clearance_m": 2,
        "camera": {
            "image_width_px": 10, "image_height_px": 10,
            "horizontal_fov_deg": 60, "vertical_fov_deg": 45,
            "pitch_deg": -90, "yaw_mode": "follow_path",
            "forward_overlap": .3, "side_overlap": .3,
        },
    }
    geometry = {"type": "Polygon", "coordinates": [
        [[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]]]}
    request = DualPlanRequest.model_validate({
        **base,
        "drones": [
            {"drone_id": "drone_1", "search_geometry": geometry,
             "home_x_m": 0, "home_y_m": 0},
            {"drone_id": "drone_2", "search_geometry": geometry,
             "home_x_m": 2, "home_y_m": 0},
        ],
    })
    assert len(request.drones) == 2
