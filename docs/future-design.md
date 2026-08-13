# Future design (Stage 6)

The following features are intentionally not implemented in version 0.1. They
must extend the core geometry API without introducing ROS, PX4, or browser
dependencies.

- Oblique cameras require ray/ground-plane projection, trapezoidal footprints,
  attitude conventions, and explicit failure when rays do not meet the ground.
- Building occlusion requires 3D building solids or meshes, camera ray tests,
  and separate ground and facade coverage metrics.
- GSD constraints should derive maximum above-ground altitude from sensor size,
  focal length, and image dimensions; they must not silently change altitude.
- Dynamic altitude and attitude require per-waypoint values and terrain data.
- A flight-controller adapter should consume the v2 continuous-flight JSON,
  track uniformly spaced ENU control points, and trigger the camera at the
  configured frequency only while the current segment has
  `capture_enabled = true`.
- YOLO visibility belongs downstream of capture and must not change deterministic
  geometric coverage unless supplied through a separately versioned policy.
