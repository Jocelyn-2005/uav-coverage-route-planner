"""Local FastAPI planning application backed by the pure-Python planner."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Literal

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
from shapely.geometry import mapping, shape

from coverage_planner.geometry.calibration import MapCalibration
from coverage_planner.io import load_semantic_map
from coverage_planner.models import CameraConfig
from coverage_planner.planner import CoveragePlanner
from coverage_planner.reporting import export_plan

ROOT = Path(__file__).resolve().parents[2]
EXAMPLE = ROOT / "examples/yungu2030"
WEB = ROOT / "web"
RESULTS = ROOT / "results/web_latest"
HOME_ENU_M = (153.4, 67.2)


class PlanRequest(BaseModel):
    search_geometry: dict[str, Any]
    home_x_m: float = HOME_ENU_M[0]
    home_y_m: float = HOME_ENU_M[1]
    flight_altitude_m: float = Field(gt=0)
    horizontal_clearance_m: float = Field(ge=0)
    vertical_clearance_m: float = Field(ge=0)
    scan_direction_deg: float | None = None
    camera: CameraConfig
    scan_pattern: Literal["lawn_mower", "contour_outward", "auto"] = "auto"
    capture_frequency_hz: float = Field(default=2.0, gt=0)
    control_point_spacing_m: float = Field(default=10.0, gt=0)
    coverage_speed_mps: float = Field(default=5.0, gt=0)
    connector_speed_mps: float = Field(default=4.0, gt=0)
    obstacle_speed_mps: float = Field(default=2.5, gt=0)
    return_speed_mps: float = Field(default=4.0, gt=0)


app = FastAPI(title="Coverage Search Planner")


@app.get("/api/map")
def map_data() -> dict[str, Any]:
    semantic = load_semantic_map(EXAMPLE / "semantic_map.json")
    calibration = MapCalibration.load(EXAMPLE / "map_calibration.json")
    image_corners = [
        calibration.pixel_to_enu(point)
        for point in ((0.0, 0.0), (1919.0, 0.0), (0.0, 1079.0), (1919.0, 1079.0))
    ]
    return {
        "world_name": semantic.world_name,
        "background": {
            "url": "/api/background",
            "bounds": [
                min(point[0] for point in image_corners),
                min(point[1] for point in image_corners),
                max(point[0] for point in image_corners),
                max(point[1] for point in image_corners),
            ],
        },
        "search_area": mapping(shape({"type":"Polygon","coordinates":[[*semantic.search_area.coords, semantic.search_area.coords[0]]]})),
        "buildings": [{"id":node.id,"height_m":node.properties.elevation_max_m,
            "bounds":[*node.shape.min_corner,*node.shape.max_corner]} for node in semantic.building_nodes],
    }


@app.get("/api/background")
def background() -> FileResponse:
    return FileResponse(EXAMPLE / "overhead_map_rotated_180.jpg", media_type="image/jpeg")


@app.post("/api/plan")
def plan(request: PlanRequest) -> dict[str, Any]:
    try:
        semantic = load_semantic_map(EXAMPLE / "semantic_map.json")
        result = CoveragePlanner().plan(
            semantic_map=semantic, search_geometry=shape(request.search_geometry), camera=request.camera,
            flight_altitude_m=request.flight_altitude_m,
            start=(request.home_x_m, request.home_y_m, request.flight_altitude_m),
            horizontal_clearance_m=request.horizontal_clearance_m,
            vertical_clearance_m=request.vertical_clearance_m,
            scan_direction_deg=request.scan_direction_deg,
            scan_pattern=request.scan_pattern,
            capture_frequency_hz=request.capture_frequency_hz,
            control_point_spacing_m=request.control_point_spacing_m,
            coverage_speed_mps=request.coverage_speed_mps,
            connector_speed_mps=request.connector_speed_mps,
            obstacle_speed_mps=request.obstacle_speed_mps,
            return_speed_mps=request.return_speed_mps,
        )
        export_plan(result, RESULTS)
        return {"summary": {
            "coverage_ratio": sum(p.area_m2*p.coverage_ratio for p in result.patches)/result.effective_area.geometry.area,
            "capture_count": sum(w.capture for w in result.waypoints),
            "transit_count": sum(not w.capture for w in result.waypoints),
            "unreachable": list(result.unreachable_patch_ids),
            "scan_pattern": result.scan_pattern,
            "path_length_m": result.path_length_m,
            "lane_count": len(result.continuous_flight.lanes) if result.continuous_flight else 0,
            "flight_waypoint_count": len(result.continuous_flight.waypoints)
                if result.continuous_flight else 0,
            "sampled_image_count": result.continuous_flight.sampled_footprint_count
                if result.continuous_flight else 0,
            "strategy_comparison": [{
                "pattern": item.pattern, "coverage_ratio": item.coverage_ratio,
                "capture_count": item.capture_waypoint_count,
                "transit_count": item.transit_waypoint_count,
                "path_length_m": item.path_length_m,
                "unreachable_count": item.unreachable_patch_count,
            } for item in result.strategy_comparison]},
            "home": [request.home_x_m, request.home_y_m, request.flight_altitude_m],
            "effective_area": mapping(result.effective_area.geometry),
            "obstacles": mapping(result.obstacles.geometry),
            "patches": [{"id":p.id,"geometry":mapping(p.geometry),"covered":p.covered,"ratio":p.coverage_ratio} for p in result.patches],
            "waypoints": [{"id":w.id,"x":w.x,"y":w.y,"z":w.z,"kind":w.kind,"capture":w.capture,
                "yaw_deg":w.yaw_deg,"covered_patch_ids":list(w.covered_patch_ids),
                "camera_footprint_enu":mapping(w.camera_footprint_enu) if w.camera_footprint_enu else None}
                for w in result.waypoints],
            "flight_waypoints": [{
                "id": w.id, "x": w.x, "y": w.y, "z": w.z,
                "heading_deg": w.heading_deg, "speed_mps": w.speed_mps,
            } for w in result.continuous_flight.waypoints] if result.continuous_flight else [],
            "route_segments": [{
                "id": segment.id, "kind": segment.kind,
                "start_waypoint_id": segment.start_waypoint_id,
                "end_waypoint_id": segment.end_waypoint_id,
                "heading_deg": segment.heading_deg, "speed_mps": segment.speed_mps,
                "capture_enabled": segment.capture_enabled,
            } for segment in result.continuous_flight.route_segments]
                if result.continuous_flight else [],
        }
    except (ValueError, TypeError) as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc


@app.get("/api/export/{filename}")
def download(filename: str) -> FileResponse:
    allowed={"waypoints.json","waypoints.csv","flight_plan.json","flight_plan.yaml",
             "patches.geojson","route.geojson","coverage_report.json","visualization.png"}
    if filename not in allowed or not (RESULTS/filename).is_file():
        raise HTTPException(status_code=404, detail="export not found")
    return FileResponse(RESULTS/filename, filename=filename)


app.mount("/", StaticFiles(directory=WEB, html=True), name="web")
