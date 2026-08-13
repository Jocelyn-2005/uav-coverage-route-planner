"""Visibility-graph routing around polygonal flight obstacles."""

from __future__ import annotations

from dataclasses import replace
from math import atan2, degrees, hypot

import networkx as nx
from shapely.geometry import LineString, MultiPolygon, Point

from coverage_planner.models.search_area import Polygonal
from coverage_planner.models.waypoint import Waypoint


class RoutingError(ValueError):
    """Raised when no collision-free route can be found."""


def route_waypoints(
    capture_waypoints: tuple[Waypoint, ...], obstacles: Polygonal,
) -> tuple[Waypoint, ...]:
    if not capture_waypoints:
        return ()
    routed = [capture_waypoints[0]]
    for destination in capture_waypoints[1:]:
        points = shortest_collision_free_path(
            (routed[-1].x, routed[-1].y), (destination.x, destination.y), obstacles
        )
        for point in points[1:-1]:
            routed.append(Waypoint(
                id="", sequence=0, kind="transit", x=point[0], y=point[1], z=destination.z,
                yaw_deg=0.0, camera_pitch_deg=destination.camera_pitch_deg, capture=False,
            ))
        routed.append(destination)
    result = []
    for index, waypoint in enumerate(routed, 1):
        next_point = routed[index] if index < len(routed) else waypoint
        yaw = _yaw((waypoint.x, waypoint.y), (next_point.x, next_point.y))
        identifier = (
            waypoint.id if waypoint.capture or waypoint.id == "wp_home_return"
            else f"wp_{index:04d}_transit"
        )
        result.append(replace(waypoint, id=identifier, sequence=index, yaw_deg=yaw))
    return tuple(result)


def route_reachable_waypoints(
    start: Waypoint,
    capture_waypoints: tuple[Waypoint, ...],
    obstacles: Polygonal,
    *,
    return_to_start: bool = False,
) -> tuple[tuple[Waypoint, ...], tuple[str, ...]]:
    """Route each capture in order while reporting unreachable destinations."""
    routed = [start]
    skipped = [
        destination.id for destination in capture_waypoints
        if obstacles.covers(Point(destination.x, destination.y))
    ]
    reachable = [
        destination for destination in capture_waypoints
        if not obstacles.covers(Point(destination.x, destination.y))
    ]
    for destination in reachable:
        try:
            points = shortest_collision_free_path(
                (routed[-1].x, routed[-1].y), (destination.x, destination.y), obstacles
            )
        except RoutingError:
            skipped.append(destination.id)
            continue
        for point in points[1:-1]:
            routed.append(Waypoint(
                id="", sequence=0, kind="transit", x=point[0], y=point[1], z=destination.z,
                yaw_deg=0.0, camera_pitch_deg=destination.camera_pitch_deg, capture=False,
            ))
        routed.append(destination)
    if return_to_start and routed[-1] is not start:
        try:
            points = shortest_collision_free_path(
                (routed[-1].x, routed[-1].y), (start.x, start.y), obstacles
            )
            for point in points[1:-1]:
                routed.append(Waypoint(
                    id="", sequence=0, kind="transit", x=point[0], y=point[1], z=start.z,
                    yaw_deg=0.0, camera_pitch_deg=start.camera_pitch_deg, capture=False,
                ))
            routed.append(replace(start, id="wp_home_return"))
        except RoutingError:
            skipped.append("wp_home_return")
    result = []
    for index, waypoint in enumerate(routed, 1):
        next_point = routed[index] if index < len(routed) else waypoint
        identifier = (
            waypoint.id if waypoint.capture or waypoint.id == "wp_home_return"
            else f"wp_{index:04d}_transit"
        )
        result.append(replace(
            waypoint, id=identifier, sequence=index,
            yaw_deg=_yaw((waypoint.x, waypoint.y), (next_point.x, next_point.y)),
        ))
    return tuple(result), tuple(skipped)


def shortest_collision_free_path(
    start: tuple[float, float], end: tuple[float, float], obstacles: Polygonal,
) -> tuple[tuple[float, float], ...]:
    if obstacles.covers(Point(start)) or obstacles.covers(Point(end)):
        raise RoutingError("route endpoint lies inside a flight-obstacle safety buffer")
    if _visible(start, end, obstacles):
        return (start, end)
    vertices = [start, end, *_vertices(obstacles)]
    graph: nx.Graph[int] = nx.Graph()
    for index, point in enumerate(vertices):
        graph.add_node(index, point=point)
    for left in range(len(vertices)):
        for right in range(left + 1, len(vertices)):
            if _visible(vertices[left], vertices[right], obstacles):
                graph.add_edge(left, right, weight=hypot(
                    vertices[right][0] - vertices[left][0],
                    vertices[right][1] - vertices[left][1],
                ))
    try:
        indices = nx.shortest_path(graph, 0, 1, weight="weight")
    except nx.NetworkXNoPath as exc:
        raise RoutingError("no collision-free route exists between waypoints") from exc
    return tuple(vertices[index] for index in indices)


def _visible(a: tuple[float, float], b: tuple[float, float], obstacles: Polygonal) -> bool:
    line = LineString([a, b])
    return line.relate(obstacles)[0] == "F"


def _vertices(obstacles: Polygonal) -> list[tuple[float, float]]:
    polygons = obstacles.geoms if isinstance(obstacles, MultiPolygon) else [obstacles]
    points: list[tuple[float, float]] = []
    for polygon in polygons:
        points.extend((float(x), float(y)) for x, y in list(polygon.exterior.coords)[:-1])
    return points


def _yaw(a: tuple[float, float], b: tuple[float, float]) -> float:
    if a == b:
        return 0.0
    return degrees(atan2(b[0] - a[0], b[1] - a[1])) % 360.0
