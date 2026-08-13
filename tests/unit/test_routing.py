import pytest
from shapely.geometry import LineString, box

from coverage_planner.models.semantic_map import SemanticMap
from coverage_planner.models.waypoint import Waypoint
from coverage_planner.routing import (
    RoutingError,
    route_reachable_waypoints,
    select_flight_obstacles,
    shortest_collision_free_path,
)


def semantic_map() -> SemanticMap:
    def node(identifier: str, x: float, height: float) -> dict[str, object]:
        return {
            "id": identifier,
            "properties": {"category": "building", "type": "building", "label": identifier,
                "passability": "restricted", "visibility": "public",
                "elevation_min_m": 0, "elevation_max_m": height},
            "shape": {"type": "rectangle", "min_corner": [x, 2], "max_corner": [x + 4, 8]},
        }
    return SemanticMap.model_validate({"schema_version": "1.0", "world_name": "routing",
        "coordinate_frame": "ENU", "units": "meters",
        "search_area": {"kind": "rectangle", "coords": [[0, 0], [30, 0], [30, 10], [0, 10]]},
        "nodes": [node("low", 5, 10), node("high", 15, 28)],
        "metadata": {"ground_truth_excluded": True, "source": "fixture"}})


def test_selects_obstacles_by_altitude_and_buffers() -> None:
    obstacles = select_flight_obstacles(
        semantic_map(), flight_altitude_m=30, vertical_clearance_m=2,
        horizontal_clearance_m=3, allow_overflight_above_buildings=True,
    )
    assert obstacles.building_ids == ("high",)
    assert obstacles.geometry.bounds == pytest.approx((12, -1, 22, 11))
    all_obstacles = select_flight_obstacles(
        semantic_map(), flight_altitude_m=100, vertical_clearance_m=2,
        horizontal_clearance_m=0, allow_overflight_above_buildings=False,
    )
    assert all_obstacles.building_ids == ("low", "high")


def test_visibility_graph_routes_around_obstacle() -> None:
    obstacle = box(4, 2, 6, 8)
    points = shortest_collision_free_path((0, 5), (10, 5), obstacle)
    assert len(points) >= 3
    for start, end in pairwise(points):
        assert LineString([start, end]).relate(obstacle)[0] == "F"


def test_direct_route_remains_direct() -> None:
    assert shortest_collision_free_path((0, 0), (10, 0), box(4, 2, 6, 8)) == ((0, 0), (10, 0))


def test_rejects_endpoint_inside_obstacle() -> None:
    with pytest.raises(RoutingError, match="endpoint"):
        shortest_collision_free_path((5, 5), (10, 5), box(4, 2, 6, 8))


def test_routes_reachable_destinations_and_reports_blocked_ones() -> None:
    start = Waypoint("start", 0, "transit", 0, 0, 10, 0, -90, False)
    blocked = Waypoint("blocked", 1, "capture", 5, 5, 10, 0, -90, True)
    reachable = Waypoint("reachable", 2, "capture", 10, 0, 10, 0, -90, True)
    route, skipped = route_reachable_waypoints(start, (blocked, reachable), box(4, 2, 6, 8))
    assert skipped == ("blocked",)
    assert any(waypoint.id == "reachable" for waypoint in route)
    assert all(waypoint.id != "blocked" for waypoint in route)


def test_closed_route_returns_to_start() -> None:
    start = Waypoint("start", 0, "transit", 0, 0, 30, 0, -90, False)
    capture = Waypoint("capture", 1, "capture", 10, 0, 30, 0, -90, True)
    route, skipped = route_reachable_waypoints(
        start, (capture,), box(4, 2, 6, 8), return_to_start=True
    )
    assert skipped == ()
    assert (route[0].x, route[0].y) == (0, 0)
    assert (route[-1].x, route[-1].y) == (0, 0)
    assert route[-1].id == "wp_home_return"
    assert not route[-1].capture
from itertools import pairwise
