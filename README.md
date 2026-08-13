# coverage-search-planner

Pure-Python UAV coverage search and obstacle-aware path planning in local ENU
metres. The project does not depend on ROS, Gazebo, PX4, MAVROS, or YOLO.

## Setup

Requires `uv`. The project pins Python 3.12 and all dependencies in `uv.lock`.

```bash
git clone https://github.com/Jocelyn-2005/coverage-search-planner.git
cd coverage-search-planner
uv sync
uv run pytest
```

Do not install dependencies into the system Python.

## Run the Yungu example

```bash
uv run coverage-planner plan \
  --semantic-map examples/yungu2030/semantic_map.json \
  --search-area examples/yungu2030/search_area.geojson \
  --config examples/yungu2030/planner_config.yaml \
  --output results/example_run
```

The command writes `waypoints.json`, `waypoints.csv`, `patches.geojson`,
`route.geojson`, `coverage_report.json`, and `visualization.png`.

## Web planner

```bash
uv run uvicorn coverage_planner.web:app --host 127.0.0.1 --port 8000
```

Open `http://127.0.0.1:8000`. The first screen is the planning workspace with
ENU map layers, mission/camera settings, planning, inspection, and exports.

## Coordinates and behavior

- All planning uses ENU metres: x East, y North, z Up.
- Pixel coordinates are used only for calibrated display and interaction.
- Searchable ground is the user polygon clipped to map bounds, minus all
  building footprints and explicit exclusions. Unlabelled outdoor ground,
  roads, paths, plazas, and green space remain searchable.
- Buildings may be flown over only when fixed altitude exceeds building height
  plus vertical clearance. The planner never raises altitude automatically.
- Version 0.1 supports nadir cameras only (`pitch_deg = -90`).

See [semantic map schema](docs/semantic-map-schema.md) and
[future design](docs/future-design.md).
