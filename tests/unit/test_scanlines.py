import pytest
from shapely.geometry import MultiPolygon, Polygon, box

from coverage_planner.coverage import generate_capture_plan
from coverage_planner.models.camera import CameraConfig


def camera() -> CameraConfig:
    return CameraConfig.model_validate({
        "image_width_px": 1000,
        "image_height_px": 1000,
        "horizontal_fov_deg": 90,
        "vertical_fov_deg": 90,
        "pitch_deg": -90,
        "yaw_mode": "follow_path",
        "forward_overlap": 0.5,
        "side_overlap": 0.5,
    })


def test_generates_deterministic_boustrophedon_waypoints() -> None:
    plan = generate_capture_plan(
        box(0, 0, 30, 30),
        camera=camera(),
        flight_altitude_m=10,
        ground_elevation_m=0,
        scan_direction_deg=90,
    )
    assert len(plan.scan_segments) == 2
    assert len(plan.capture_waypoints) == 4
    expected_points = [(10, 10), (20, 10), (20, 20), (10, 20)]
    for waypoint, expected in zip(plan.capture_waypoints, expected_points, strict=True):
        assert (waypoint.x, waypoint.y) == pytest.approx(expected)
    assert [waypoint.yaw_deg for waypoint in plan.capture_waypoints] == [90, 90, 270, 270]
    assert [waypoint.id for waypoint in plan.capture_waypoints] == [
        "wp_0001", "wp_0002", "wp_0003", "wp_0004",
    ]


def test_scanline_intersections_split_around_hole() -> None:
    effective = Polygon(
        [(0, 0), (50, 0), (50, 30), (0, 30)],
        holes=[[(20, 2), (30, 2), (30, 28), (20, 28)]],
    )
    plan = generate_capture_plan(
        effective,
        camera=camera(),
        flight_altitude_m=5,
        ground_elevation_m=0,
        scan_direction_deg=90,
    )
    assert len(plan.scan_segments) == 10
    assert {segment.segment_index for segment in plan.scan_segments} == {0, 1}
    for waypoint in plan.capture_waypoints:
        assert effective.covers(box(waypoint.x, waypoint.y, waypoint.x, waypoint.y))
        assert not (20 < waypoint.x < 30)


def test_supports_disconnected_components() -> None:
    effective = MultiPolygon([box(0, 0, 20, 20), box(40, 0, 60, 20)])
    plan = generate_capture_plan(
        effective,
        camera=camera(),
        flight_altitude_m=5,
        ground_elevation_m=0,
        scan_direction_deg=90,
    )
    assert len(plan.scan_segments) == 6
    assert all(waypoint.x <= 20 or waypoint.x >= 40 for waypoint in plan.capture_waypoints)


def test_narrow_region_uses_single_centered_scanline_and_capture() -> None:
    plan = generate_capture_plan(
        box(10, 20, 14, 23),
        camera=camera(),
        flight_altitude_m=10,
        ground_elevation_m=0,
        scan_direction_deg=90,
    )
    assert len(plan.scan_segments) == 1
    assert len(plan.capture_waypoints) == 1
    waypoint = plan.capture_waypoints[0]
    assert (waypoint.x, waypoint.y) == pytest.approx((12, 21.5))


def test_rotated_scan_direction_preserves_capture_points_inside_geometry() -> None:
    effective = box(0, 0, 40, 30)
    plan = generate_capture_plan(
        effective,
        camera=camera(),
        flight_altitude_m=5,
        ground_elevation_m=0,
        scan_direction_deg=45,
    )
    assert plan.scan_direction_deg == 45
    assert plan.capture_waypoints
    assert all(
        effective.covers(box(waypoint.x, waypoint.y, waypoint.x, waypoint.y))
        for waypoint in plan.capture_waypoints
    )


def test_same_input_produces_identical_plan() -> None:
    arguments = {
        "camera": camera(),
        "flight_altitude_m": 5,
        "ground_elevation_m": 0,
        "scan_direction_deg": 15,
    }
    first = generate_capture_plan(box(-5, -8, 34, 27), **arguments)  # type: ignore[arg-type]
    second = generate_capture_plan(box(-5, -8, 34, 27), **arguments)  # type: ignore[arg-type]
    assert first == second
