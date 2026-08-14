"""Boustrophedon cellular decomposition coverage generator."""

from __future__ import annotations

from dataclasses import dataclass, replace
from itertools import pairwise

from shapely import affinity, unary_union
from shapely.geometry import GeometryCollection, MultiPolygon, Polygon, box

from coverage_planner.coverage.scanlines import CapturePlan, generate_capture_plan
from coverage_planner.models.camera import CameraConfig
from coverage_planner.models.search_area import Polygonal
from coverage_planner.models.waypoint import ScanSegment, Waypoint


@dataclass(frozen=True, slots=True)
class BCDGenerator:
    """Decompose free ground into sweep-monotone cells, then lane each cell."""

    method: str = "bcd"

    def generate(
        self,
        geometry: Polygonal,
        *,
        camera: CameraConfig,
        flight_altitude_m: float,
        ground_elevation_m: float,
        scan_direction_deg: float,
    ) -> CapturePlan:
        cells = decompose_boustrophedon_cells(
            geometry, scan_direction_deg=scan_direction_deg)
        waypoints: list[Waypoint] = []
        segments: list[ScanSegment] = []
        lane_index = 0
        for cell in cells:
            local = generate_capture_plan(
                cell,
                camera=camera,
                flight_altitude_m=flight_altitude_m,
                ground_elevation_m=ground_elevation_m,
                scan_direction_deg=scan_direction_deg,
            )
            by_id = {waypoint.id: waypoint for waypoint in local.capture_waypoints}
            for local_segment in local.scan_segments:
                lane_waypoints = [by_id[item] for item in local_segment.capture_waypoint_ids]
                ids: list[str] = []
                for waypoint in lane_waypoints:
                    waypoint_id = f"wp_{len(waypoints) + 1:04d}"
                    ids.append(waypoint_id)
                    waypoints.append(replace(
                        waypoint,
                        id=waypoint_id,
                        sequence=len(waypoints) + 1,
                        scan_line_index=lane_index,
                        scan_segment_index=0,
                    ))
                segments.append(ScanSegment(
                    scan_line_index=lane_index,
                    segment_index=0,
                    start_enu_m=local_segment.start_enu_m,
                    end_enu_m=local_segment.end_enu_m,
                    direction_yaw_deg=local_segment.direction_yaw_deg,
                    capture_waypoint_ids=tuple(ids),
                ))
                lane_index += 1
        return CapturePlan(scan_direction_deg % 360.0, tuple(segments), tuple(waypoints))


def decompose_boustrophedon_cells(
    geometry: Polygonal, *, scan_direction_deg: float,
) -> tuple[Polygonal, ...]:
    """Return maximal cells between sweep events without topology branching.

    Geometry is rotated so coverage lanes are horizontal. Vertex ordinates form
    sweep events. Adjacent slab fragments are merged only across one-to-one
    connectivity; a split or merge starts new cells, which is the defining BCD
    topology rule.
    """
    if geometry.is_empty:
        return ()
    rotation_deg = scan_direction_deg % 360.0 - 90.0
    rotated = affinity.rotate(geometry, rotation_deg, origin=(0.0, 0.0))
    event_y = sorted({coordinate[1] for polygon in _polygon_parts(rotated)
                      for ring in (polygon.exterior, *polygon.interiors)
                      for coordinate in ring.coords})
    if len(event_y) < 2:
        return (geometry,)
    min_x, _, max_x, _ = rotated.bounds
    margin = max(1.0, max_x - min_x)
    slabs: list[list[Polygon]] = []
    for lower, upper in pairwise(event_y):
        if upper - lower <= 1e-9:
            continue
        sliced = rotated.intersection(box(min_x - margin, lower, max_x + margin, upper))
        parts = sorted(_polygon_parts(sliced), key=lambda item: (item.centroid.x, item.area))
        if parts:
            slabs.append(parts)
    if not slabs:
        return tuple(_rotate_back(item, rotation_deg) for item in _polygon_parts(rotated))

    fragments = [item for slab in slabs for item in slab]
    fragment_index = {id(item): index for index, item in enumerate(fragments)}
    parent = list(range(len(fragments)))

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def union(left: int, right: int) -> None:
        left_root, right_root = find(left), find(right)
        if left_root != right_root:
            parent[right_root] = left_root

    for previous, following in pairwise(slabs):
        links = [(left, right) for left in previous for right in following
                 if left.boundary.intersection(right.boundary).length > 1e-8]
        left_degree = {id(item): 0 for item in previous}
        right_degree = {id(item): 0 for item in following}
        for left, right in links:
            left_degree[id(left)] += 1
            right_degree[id(right)] += 1
        for left, right in links:
            if left_degree[id(left)] == 1 and right_degree[id(right)] == 1:
                union(fragment_index[id(left)], fragment_index[id(right)])

    groups: dict[int, list[Polygon]] = {}
    for index, fragment in enumerate(fragments):
        groups.setdefault(find(index), []).append(fragment)
    cells: list[Polygonal] = []
    for parts in groups.values():
        merged = unary_union(parts)
        cells.extend(_rotate_back(item, rotation_deg) for item in _polygon_parts(merged))
    return tuple(sorted(cells, key=lambda item: (item.centroid.y, item.centroid.x, item.area)))


def _rotate_back(polygon: Polygon, rotation_deg: float) -> Polygon:
    result = affinity.rotate(polygon, -rotation_deg, origin=(0.0, 0.0))
    if not isinstance(result, Polygon):
        raise TypeError("rotating a polygon must produce a polygon")
    return result


def _polygon_parts(geometry: object) -> list[Polygon]:
    if isinstance(geometry, Polygon):
        return [geometry] if geometry.area > 1e-9 else []
    if isinstance(geometry, MultiPolygon):
        return [item for item in geometry.geoms if item.area > 1e-9]
    if isinstance(geometry, GeometryCollection):
        return [item for item in geometry.geoms
                if isinstance(item, Polygon) and item.area > 1e-9]
    return []
