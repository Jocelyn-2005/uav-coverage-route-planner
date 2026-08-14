# Future design (Stage 6)

The following features are intentionally not implemented in version 0.1. They
must extend the core geometry API without introducing ROS, PX4, or browser
dependencies.

- Oblique cameras require ray/ground-plane projection, trapezoidal footprints,
  attitude conventions, and explicit failure when rays do not meet the ground.
- The current 2.5D rectangular-building shadow model should later be upgraded to
  arbitrary 3D solids or meshes and explicit camera-ray tests. Facade coverage
  is intentionally outside the ground-target mission.
- GSD constraints should derive maximum above-ground altitude from sensor size,
  focal length, and image dimensions; they must not silently change altitude.
- Dynamic altitude and attitude require per-waypoint values and terrain data.
- A flight-controller adapter should consume the v3 continuous-video JSON and
  track the bounded-spacing ENU control points. The camera supplies a continuous
  stream; `detection_enabled` controls whether detections count toward the task.
- Visual recognition belongs downstream of planning and must not change the
  deterministic geometric coverage model unless supplied through a separately
  versioned policy.
