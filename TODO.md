# Future improvements

- Derive or correct the robot's heading estimate from left, front, and right sensor geometry instead of relying only on integrated motion.
- Add sensor-noise modelling and a Kalman-filter-based fusion layer for pose estimation.
- Fuse commanded velocity, measured velocity, and sensor-derived pose corrections while preserving the current control-layer abstraction.

These improvements should be considered only after the basic pose, MotionState, and kinematic control layers are reliable.
