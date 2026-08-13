"""Deterministic rectangular ground projection for a nadir camera."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, radians, sin, tan

from shapely.geometry import Polygon

from coverage_planner.models.camera import CameraConfig


class CameraGeometryError(ValueError):
    """Raised when a valid ground footprint cannot be calculated."""


@dataclass(frozen=True, slots=True)
class GroundFootprintDimensions:
    """Camera coverage and waypoint spacing, expressed in metres."""

    width_m: float
    length_m: float
    capture_spacing_m: float
    scan_line_spacing_m: float
    height_above_ground_m: float


def ground_footprint_dimensions(
    camera: CameraConfig,
    *,
    flight_altitude_m: float,
    ground_elevation_m: float,
) -> GroundFootprintDimensions:
    """Calculate nadir footprint dimensions at a horizontal ground plane."""
    height_m = flight_altitude_m - ground_elevation_m
    if height_m <= 0.0:
        raise CameraGeometryError(
            "flight_altitude_m must be greater than ground_elevation_m for a nadir footprint"
        )
    width_m = 2.0 * height_m * tan(radians(camera.horizontal_fov_deg) / 2.0)
    length_m = 2.0 * height_m * tan(radians(camera.vertical_fov_deg) / 2.0)
    return GroundFootprintDimensions(
        width_m=width_m,
        length_m=length_m,
        capture_spacing_m=length_m * (1.0 - camera.forward_overlap),
        scan_line_spacing_m=width_m * (1.0 - camera.side_overlap),
        height_above_ground_m=height_m,
    )


def ground_footprint_polygon(
    camera: CameraConfig,
    *,
    center_enu_m: tuple[float, float],
    flight_altitude_m: float,
    ground_elevation_m: float,
    yaw_deg: float,
) -> Polygon:
    """Project a rectangular footprint with yaw clockwise from ENU North."""
    dimensions = ground_footprint_dimensions(
        camera,
        flight_altitude_m=flight_altitude_m,
        ground_elevation_m=ground_elevation_m,
    )
    yaw_rad = radians(yaw_deg)
    forward = (sin(yaw_rad), cos(yaw_rad))
    right = (cos(yaw_rad), -sin(yaw_rad))
    half_width = dimensions.width_m / 2.0
    half_length = dimensions.length_m / 2.0
    center_x, center_y = center_enu_m

    corners = []
    for forward_sign, right_sign in ((1.0, 1.0), (1.0, -1.0), (-1.0, -1.0), (-1.0, 1.0)):
        corners.append((
            center_x
            + forward_sign * half_length * forward[0]
            + right_sign * half_width * right[0],
            center_y
            + forward_sign * half_length * forward[1]
            + right_sign * half_width * right[1],
        ))
    return Polygon(corners)
