from pathlib import Path

import pytest

from coverage_planner.geometry.calibration import MapCalibration
from coverage_planner.io.semantic_map import load_semantic_map, search_area_geometry
from coverage_planner.visualization import render_semantic_map
from coverage_planner.visualization.semantic_map import semantic_map_display_bounds

EXAMPLE = Path("examples/yungu2030/semantic_map.json")


def test_loads_real_yungu_semantic_map() -> None:
    semantic_map = load_semantic_map(EXAMPLE)
    assert semantic_map.world_name == "yungu2030_local_origin"
    assert len(semantic_map.nodes) == 43
    assert len(semantic_map.building_nodes) == 25
    assert search_area_geometry(semantic_map).bounds == (
        -4.835386753082275, -10.610369682312012, 324.72998046875, 210.56312561035156,
    )


def test_renders_real_yungu_map(tmp_path: Path) -> None:
    output = render_semantic_map(load_semantic_map(EXAMPLE), tmp_path / "overview.png")
    assert output.stat().st_size > 10_000


def test_real_overhead_map_calibration_and_overlay(tmp_path: Path) -> None:
    calibration = MapCalibration.load(EXAMPLE.parent / "map_calibration.json")
    assert calibration.pixel_to_enu((269.15, 1026.25)) == pytest.approx((0.0, 0.0), abs=1e-8)
    assert calibration.pixel_to_enu((1678.555, 89.75)) == pytest.approx(
        (305.0, 200.0), abs=1e-8
    )
    assert semantic_map_display_bounds(
        load_semantic_map(EXAMPLE),
        image_size_px=(1920, 1080),
        calibration=calibration,
    ) == pytest.approx((-58.245, -11.265, 357.033, 219.167), abs=0.001)
    output = render_semantic_map(
        load_semantic_map(EXAMPLE),
        tmp_path / "overlay.png",
        background_image_path=EXAMPLE.parent / "overhead_map_rotated_180.jpg",
        calibration=calibration,
    )
    assert output.stat().st_size > 100_000
