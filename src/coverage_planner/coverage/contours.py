"""Center-outward contour coverage generation."""

from __future__ import annotations

from math import atan2, ceil, degrees, hypot

from shapely.geometry import GeometryCollection, LineString, MultiLineString, Point

from coverage_planner.camera import ground_footprint_dimensions, ground_footprint_polygon
from coverage_planner.coverage.scanlines import CapturePlan
from coverage_planner.models.camera import CameraConfig
from coverage_planner.models.search_area import Polygonal
from coverage_planner.models.waypoint import ScanSegment, Waypoint


def generate_contour_capture_plan(
    effective_geometry: Polygonal,
    *,
    camera: CameraConfig,
    flight_altitude_m: float,
    ground_elevation_m: float,
    center_enu_m: tuple[float, float],
) -> CapturePlan:
    """Sample clipped concentric contours, ordered from the mission center outward."""
    if effective_geometry.is_empty:
        return CapturePlan(0.0, (), ())
    dimensions = ground_footprint_dimensions(
        camera,
        flight_altitude_m=flight_altitude_m,
        ground_elevation_m=ground_elevation_m,
    )
    center = Point(center_enu_m)
    min_x, min_y, max_x, max_y = effective_geometry.bounds
    maximum_radius_m = max(
        hypot(x - center.x, y - center.y)
        for x, y in ((min_x, min_y), (min_x, max_y), (max_x, min_y), (max_x, max_y))
    )
    spacing_m = dimensions.scan_line_spacing_m
    radii = [spacing_m / 2.0 + index * spacing_m
             for index in range(ceil(maximum_radius_m / spacing_m) + 1)]

    waypoints: list[Waypoint] = []
    segments: list[ScanSegment] = []
    current = center_enu_m
    for layer_index, radius_m in enumerate(radii):
        contour = center.buffer(radius_m, quad_segs=12).boundary
        remaining = _line_parts(contour.intersection(effective_geometry))
        segment_index = 0
        while remaining:
            part_index, reverse = min(
                ((index, end == 1) for index, part in enumerate(remaining) for end in (0, 1)),
                key=lambda item: (
                    Point(current).distance(Point(list(remaining[item[0]].coords)[-1 if item[1] else 0])),
                    item[0], item[1],
                ),
            )
            part = remaining.pop(part_index)
            if reverse:
                part = LineString(reversed(part.coords))
            points = _sample_line(part, dimensions.capture_spacing_m)
            ids: list[str] = []
            for point_index, point in enumerate(points):
                neighbor = points[min(point_index + 1, len(points) - 1)]
                if neighbor == point and point_index:
                    neighbor = points[point_index - 1]
                yaw_deg = degrees(atan2(neighbor[1] - point[1], neighbor[0] - point[0])) % 360.0
                waypoint_id = f"wp_{len(waypoints) + 1:04d}"
                ids.append(waypoint_id)
                waypoints.append(Waypoint(
                    id=waypoint_id, sequence=len(waypoints) + 1, kind="capture",
                    x=point[0], y=point[1], z=flight_altitude_m, yaw_deg=yaw_deg,
                    camera_pitch_deg=camera.pitch_deg, capture=True,
                    scan_line_index=layer_index, scan_segment_index=segment_index,
                    camera_footprint_enu=ground_footprint_polygon(
                        camera, center_enu_m=point, flight_altitude_m=flight_altitude_m,
                        ground_elevation_m=ground_elevation_m, yaw_deg=yaw_deg,
                    ),
                ))
            if points:
                segments.append(ScanSegment(
                    scan_line_index=layer_index, segment_index=segment_index,
                    start_enu_m=points[0], end_enu_m=points[-1],
                    direction_yaw_deg=waypoints[-len(points)].yaw_deg,
                    capture_waypoint_ids=tuple(ids),
                ))
                current = points[-1]
                segment_index += 1
    return CapturePlan(0.0, tuple(segments), tuple(waypoints))


def _sample_line(line: LineString, maximum_spacing_m: float) -> list[tuple[float, float]]:
    count = max(1, ceil(line.length / maximum_spacing_m))
    return [
        (float(point.x), float(point.y))
        for point in (line.interpolate(index / count, normalized=True) for index in range(count + 1))
    ]


def _line_parts(geometry: object) -> list[LineString]:
    if isinstance(geometry, LineString):
        return [geometry] if geometry.length > 0 else []
    if isinstance(geometry, MultiLineString):
        return [line for line in geometry.geoms if line.length > 0]
    if isinstance(geometry, GeometryCollection):
        return [part for part in geometry.geoms if isinstance(part, LineString) and part.length > 0]
    return []
