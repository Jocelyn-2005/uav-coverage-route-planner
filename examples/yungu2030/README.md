# Yungu2030 example data

`semantic_map.json` is copied from the original GSI project's
`data/yungu2030_v1/semantic_map.json`. It is a small, runtime-independent ENU
semantic asset used by the planner and integration tests.

`search_area.geojson` is an ENU GeoJSON copy of the semantic map's full search
boundary. It provides a directly runnable user-search input while preserving
the semantic JSON as the source of truth.

`planner_config.yaml` contains reproducible example camera, altitude, overlap,
clearance, start, and scan-direction settings for the CLI and Web planner.

The original directory contains a 228 MB 3D GLB mesh. The mesh and all
ROS/Gazebo/PX4 files are intentionally excluded from this pure-Python
repository.

The supplied 1920 x 1080 overhead JPEG has an exported grey border and its raw
orientation is opposite to the semantic ENU map. `overhead_map_rotated_180.jpg`
is the canonical planning image after a 180-degree rotation. In that image,
right and up correspond to ENU East and North. The affine calibration was
refined against roof edges distributed over all 25 building collision
rectangles, rather than the exported grey image border. ENU `(0, 0)` maps to
approximately image pixel `(269.15, 1026.25)`. The fitted image scales are
approximately 4.621 pixels per East metre and 4.6825 pixels per North metre.

The semantic `search_area` is larger than the image coverage: its negative x/y
margin and far northern strip extend outside the supplied raster. It must not be
forced onto the image edges. Building nodes are broad axis-aligned collision
boxes rather than precise roof outlines, so their edges need not follow every
roof indentation.
