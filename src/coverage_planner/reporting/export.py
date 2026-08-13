"""Deterministic mission artifact export."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib
import yaml

matplotlib.use("Agg")
from matplotlib import pyplot as plt
from shapely.geometry import LineString, mapping

from coverage_planner import __version__
from coverage_planner.planner import PlanResult


def export_plan(result: PlanResult, output_dir: str | Path) -> Path:
    output = Path(output_dir); output.mkdir(parents=True, exist_ok=True)
    waypoint_rows = [_waypoint(wp) for wp in result.waypoints]
    summary = _summary(result)
    mission = {"schema_version":"1.0", "coordinate_frame":"ENU", "units":"meters",
        "map_id":result.semantic_map.world_name,
        "planner":{"name":"coverage_planner","version":__version__},
        "waypoints":waypoint_rows, "summary":summary}
    _json(output/"waypoints.json", mission)
    if result.continuous_flight is not None:
        flight_mission = _flight_mission(result)
        _json(output/"flight_plan.json", flight_mission)
        (output/"flight_plan.yaml").write_text(
            yaml.safe_dump(flight_mission, sort_keys=False, allow_unicode=True), encoding="utf-8")
    with (output/"waypoints.csv").open("w", newline="", encoding="utf-8") as stream:
        writer=csv.DictWriter(stream, fieldnames=["id","sequence","kind","x","y","z","yaw_deg","capture"])
        writer.writeheader(); writer.writerows({k:v for k,v in row.items() if k in writer.fieldnames} for row in waypoint_rows)
    _json(output/"patches.geojson", {"type":"FeatureCollection","features":[
        {"type":"Feature","geometry":mapping(p.geometry),"properties":{"id":p.id,"area_m2":p.area_m2,
         "covered":p.covered,"coverage_ratio":p.coverage_ratio}} for p in result.patches]})
    route = LineString([(w.x,w.y) for w in result.waypoints]) if len(result.waypoints)>1 else LineString()
    _json(output/"route.geojson", {"type":"Feature","geometry":mapping(route),"properties":summary})
    _json(output/"coverage_report.json", summary | {"warnings":list(result.warnings)})
    _visualization(result, output/"visualization.png")
    return output


def _waypoint(wp: object) -> dict[str, object]:
    from coverage_planner.models.waypoint import Waypoint
    assert isinstance(wp, Waypoint)
    return {"id":wp.id,"sequence":wp.sequence,"kind":wp.kind,"x":wp.x,"y":wp.y,"z":wp.z,
            "yaw_deg":wp.yaw_deg,"camera_pitch_deg":wp.camera_pitch_deg,"capture":wp.capture,
            "covered_patch_ids":list(wp.covered_patch_ids),
            "camera_footprint_enu":mapping(wp.camera_footprint_enu) if wp.camera_footprint_enu else None}


def _summary(result: PlanResult) -> dict[str, object]:
    covered=sum(p.area_m2*p.coverage_ratio for p in result.patches); effective=result.effective_area.geometry.area
    segment_lengths: dict[str, float] = {}
    if result.continuous_flight is not None:
        for segment in result.continuous_flight.route_segments:
            segment_lengths[segment.kind] = segment_lengths.get(segment.kind, 0.0) + segment.length_m
    noncoverage = sum(value for key, value in segment_lengths.items() if key != "coverage_lane")
    return {"capture_waypoint_count":sum(w.capture for w in result.waypoints),
      "transit_waypoint_count":sum(not w.capture for w in result.waypoints),"path_length_m":result.path_length_m,
      "total_requested_area_m2":result.effective_area.metrics.requested_area_m2,
      "effective_search_area_m2":effective,"building_excluded_area_m2":result.effective_area.metrics.building_excluded_area_m2,
      "covered_area_m2":covered,"uncovered_area_m2":max(0,effective-covered),
      "coverage_ratio":covered/effective if effective else 0,"scan_direction_deg":result.scan_direction_deg,
      "scan_pattern":result.scan_pattern,
      "strategy_comparison":[{
          "pattern": item.pattern, "coverage_ratio": item.coverage_ratio,
          "capture_waypoint_count": item.capture_waypoint_count,
          "transit_waypoint_count": item.transit_waypoint_count,
          "path_length_m": item.path_length_m,
          "unreachable_patch_count": item.unreachable_patch_count,
      } for item in result.strategy_comparison],
      "deadhead_distance_m":noncoverage,
      "turn_count":max(0, len(result.continuous_flight.lanes)-1)
          if result.continuous_flight else 0,
      "minimum_obstacle_clearance_m":None,
      "coverage_lane_length_m":segment_lengths.get("coverage_lane", 0.0),
      "connector_length_m":segment_lengths.get("connector", 0.0),
      "obstacle_avoidance_length_m":segment_lengths.get("obstacle_avoidance", 0.0),
      "return_home_length_m":segment_lengths.get("return_home", 0.0),
      "noncoverage_distance_ratio":noncoverage/result.path_length_m if result.path_length_m else 0.0,
      "sampled_image_count":result.continuous_flight.sampled_footprint_count
          if result.continuous_flight else 0,
      "unreachable_patch_ids":list(result.unreachable_patch_ids)}


def _flight_mission(result: PlanResult) -> dict[str, object]:
    flight = result.continuous_flight
    assert flight is not None
    return {
        "schema_version": "2.0", "coordinate_frame": "ENU", "units": "meters",
        "map_id": result.semantic_map.world_name,
        "capture": {"mode": "continuous", "frequency_hz": flight.capture_frequency_hz,
                    "control_point_spacing_m": flight.control_point_spacing_m,
                    "forward_overlap": flight.forward_overlap,
                    "lane_overlap": flight.lane_overlap},
        "lanes": [{
            "id": lane.id, "sequence": lane.sequence, "heading_deg": lane.heading_deg,
            "speed_mps": lane.speed_mps, "route_segment_ids": list(lane.route_segment_ids),
            "length_m": lane.length_m,
        } for lane in flight.lanes],
        "route_segments": [{
            "id": segment.id, "sequence": segment.sequence, "kind": segment.kind,
            "start_waypoint_id": segment.start_waypoint_id,
            "end_waypoint_id": segment.end_waypoint_id,
            "heading_deg": segment.heading_deg, "speed_mps": segment.speed_mps,
            "length_m": segment.length_m, "capture_enabled": segment.capture_enabled,
        } for segment in flight.route_segments],
        "waypoints": [{
            "id": waypoint.id, "sequence": waypoint.sequence,
            "x": waypoint.x, "y": waypoint.y, "z": waypoint.z,
            "heading_deg": waypoint.heading_deg, "speed_mps": waypoint.speed_mps,
        } for waypoint in flight.waypoints],
        "summary": _summary(result),
    }


def _json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")


def _visualization(result: PlanResult, path: Path) -> None:
    figure, axes = plt.subplots(figsize=(12, 8))
    for patch in result.patches:
        geometries = patch.geometry.geoms if patch.geometry.geom_type == "MultiPolygon" else [patch.geometry]
        for geometry in geometries:
            x, y = geometry.exterior.xy
            axes.fill(x, y, color="#4f9d69" if patch.covered else "#d95d5d", alpha=0.35)
    if result.waypoints:
        axes.plot([w.x for w in result.waypoints], [w.y for w in result.waypoints], color="#1864ab", linewidth=.8)
        axes.scatter([w.x for w in result.waypoints if w.capture], [w.y for w in result.waypoints if w.capture], s=8, color="#111111")
    axes.set_aspect("equal"); axes.set_xlabel("East x (m)"); axes.set_ylabel("North y (m)")
    axes.set_title(f"{result.semantic_map.world_name} coverage plan"); figure.tight_layout()
    figure.savefig(path, dpi=160); plt.close(figure)
