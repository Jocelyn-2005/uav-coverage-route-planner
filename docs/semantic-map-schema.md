# Observed Yungu2030 semantic-map schema

The source asset was inspected before implementing the models. Its top-level
keys are `schema_version`, `world_name`, `coordinate_frame`, `units`,
`search_area`, `nodes`, and `metadata`.

- The map declares schema `1.0`, frame `ENU`, and units `meters`.
- `search_area` is a four-coordinate rectangle. Its actual bounds are read from
  the file, never hardcoded by the planner.
- There are 43 nodes: 25 buildings, 14 areas, and 4 transport facilities.
- Every observed node has a rectangular shape with `min_corner` and
  `max_corner` in ENU x/y metres.
- Every node property contains `category`, `type`, `label`, `passability`,
  `visibility`, `elevation_min_m`, and `elevation_max_m`.
- Metadata records whether ground truth is excluded and the source asset.

The current parser intentionally validates this observed format strictly. New
shape types or schema versions should be added only after inspecting real input
that uses them.
