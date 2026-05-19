# Known Limitations

These are limitations of the current implementation, not bugs. Each entry includes why it exists and what the workaround or planned fix is.

---

## GPS blackouts longer than 5-7 minutes cause heading drift

During GPS blackouts, FusionCore dead-reckons on IMU and wheel encoders. The encoder WZ bias (`B_EWZ`) is calibrated from GPS heading cross-covariance before the blackout and subtracted during it. For blackouts up to a few minutes, this works well. Beyond 5-7 minutes, the residual uncorrected heading rate error (thermal drift in `B_EWZ`, temperature-dependent gyro drift) accumulates into position error quadratically.

On the NCLT 2012-06-15 sequence (461-second blackout), FC accumulated significant heading error. See [issue #63](https://github.com/manankharwar/fusioncore/issues/63).

**Workaround:** If your platform has a magnetometer and your operating environment has low magnetic interference, fusing magnetometer heading during blackouts closes this observability gap. Magnetometer support is on the roadmap.

---

## Adversarial GPS outlier clusters at blackout boundaries can corrupt the estimate

FusionCore's chi2 gate relaxes during GPS blackout recovery (`coast_q_factor`) to accept valid returning fixes after the predicted covariance inflates. If a cluster of corrupt GPS fixes arrives at exactly the re-acquisition moment, the relaxed gate may accept them before the filter's state recovers.

This pattern appears in NCLT 2012-08-20: 105 mode-3 GPS fixes 720-840 m off RTK arrive in a 24-second window at the end of a 211-second blackout. The filter accepts them, spikes to ~788 m error, and recovers within 2 minutes. See [issue #64](https://github.com/manankharwar/fusioncore/issues/64).

**Workaround:** Lower `gnss.coast_q_factor` to tighten the gate during recovery. This reduces the spike magnitude at the cost of slower re-acquisition after legitimate long blackouts. There is no single value that handles both cases optimally.

**Not yet implemented: velocity sanity check.** A GPS fix 720 m from the dead-reckoned position after a 211-second blackout implies ~3400 m/s of motion. A hard `max_implied_speed` check (e.g., 20 m/s) operating before the chi2 gate rejects this pattern trivially and has zero effect on normal operation. This check is tracked but not yet in the filter.

**Not yet implemented: cluster consistency gate.** A single outlier at 720 m is handled by chi2. Five consecutive fixes all landing 720-840 m from the predicted position with tight geometric consistency is a distinguishable pattern from noise. A secondary check on cluster coherence across multiple consecutive fixes would catch this without affecting single-fix rejection behavior. This is the planned architectural fix.

---

## IMU + encoder alone cannot separate gyro bias from encoder bias without GPS

When GPS is unavailable and both sensors are active, the system has three unknowns (WZ, B_GZ, B_EWZ) observable from two measurement equations. The system is rank-deficient.

FusionCore resolves this by treating `B_EWZ` as known from the GPS calibration phase (tight prior on `P(B_EWZ, B_EWZ)`). This works correctly during GPS blackouts when `B_EWZ` was calibrated before the blackout. It does not work if GPS was never available: `B_EWZ` remains unobservable, and the filter cannot separate the two biases.

**Workaround:** Ensure the robot operates with GPS for at least the first minute of each run to calibrate `B_EWZ`. The `init.stationary_window` setting helps calibrate `B_GZ` before motion begins.

---

## Non-holonomic constraint assumes flat ground

The NHC (non-holonomic constraint) zeros the lateral and vertical body-frame velocity as a virtual measurement. This assumes the robot is on flat ground. On significant slopes or when traversing obstacles, the vertical velocity constraint produces an incorrect measurement that the filter must reject or accommodate.

**Workaround:** Reduce `encoder.nhc_noise_vy` and `encoder.nhc_noise_vz` to loosen the constraint (higher values = weaker constraint = less penalty for non-zero lateral/vertical velocity). On robots that regularly traverse steep terrain, set these to very large values to effectively disable the constraint.

---

## IMU ring buffer retrodiction assumes IMU is the highest-rate sensor

The delay compensation mechanism buffers up to 1 second of IMU measurements (at `imu.topic` rate) and replays them when a delayed GPS fix arrives. This assumes IMU arrives at the filter's nominal rate and GPS arrives late. If GPS arrives before IMU (e.g., low-frequency IMU with high-frequency GPS), the buffer does not help.

**Limitation:** The buffer covers GPS latency up to 500 ms by default (`gnss.max_delay_s`). Fixes arriving more than 500 ms late are processed at arrival time with no retrodiction.

---

## Single global config, no per-session adaptation

FusionCore's adaptive noise covariance updates within a session from the innovation sequence. It does not persist learned noise parameters across sessions or adapt its base parameters based on environment type (urban canyon, open field, indoor/outdoor).

**Workaround:** Maintain separate config files for meaningfully different operating environments and select the appropriate one at launch.

---

## VSLAM map frame assumed stationary

When fusing VSLAM pose (`vslam.topic`), FusionCore anchors the offset between the VSLAM map frame and the filter's odometry frame on first message. If the VSLAM system re-localizes to a different part of its map (loop closure, kidnapping recovery), the map origin shifts and the anchored offset becomes stale.

The reinitialization detector (`vslam.reinit_n`, `vslam.reinit_jump_m`) catches large instantaneous jumps. It does not catch slow drift due to map deformation.

**Workaround:** Trigger a filter reset (`ros2 service call /fusioncore/reset`) after VSLAM re-localization completes.
