from shapely.geometry import Polygon

from coverage_planner.coverage.optimization import prepare_lane_route
from coverage_planner.coverage.scanlines import CapturePlan
from coverage_planner.models import Waypoint


def point(identifier: str, sequence: int, x: float, y: float,
          line: int, segment: int = 0) -> Waypoint:
    return Waypoint(
        identifier, sequence, "capture", x, y, 25, 0, -90, True,
        line, segment,
    )


def test_lane_route_keeps_only_endpoints_and_chooses_near_orientation() -> None:
    plan = CapturePlan(90, (), (
        point("a", 1, 0, 0, 0),
        point("b", 2, 5, 0, 0),
        point("c", 3, 10, 0, 0),
        point("d", 4, 0, 10, 1),
        point("e", 5, 5, 10, 1),
        point("f", 6, 10, 10, 1),
    ))
    route, skipped = prepare_lane_route(
        plan, start_enu_m=(11, 0), obstacles=Polygon())
    assert skipped == ()
    assert [(item.x, item.y) for item in route] == [
        (10, 0), (0, 0), (0, 10), (10, 10)]
    assert [item.id for item in route] == [
        "wp_0001", "wp_0002", "wp_0003", "wp_0004"]
