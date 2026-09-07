# C++ controller core

`Qarm::Controller` is a C++14 in-process state machine backed by the existing
`QminiArm::Core` conversion, gravity, torque limiting, feedback safety and
trajectory validation functions. No daemon, JSON parser, Unix socket or Python
binding is implemented here. The Python fake service is a separate adapter;
it does not invoke this C++ class yet.

Construct `ArmController(ControllerConfig, Clock)` with the current model hash,
board boot ID, motor map and verified limits. The default numeric limits are
conservative fixtures, not a deployment configuration loader. The gravity
model is the existing compiled Qmini four-axis model; model hash generation and
binding that model to a deployment belong to the eventual service launcher.
`Clock` defaults to `steady_clock` nanoseconds and can be injected in tests.

The public API is `connect`, `disconnect`, `handleCommand`, `updateFeedback`,
`tick`, `reportFault`, `snapshot` and `output`. Both state and output accessors
return copies. A service queues frontend commands and dispatches them on its
control thread. `ControlOutput` is local bus data, never a frontend command.

The capture path requires an exclusive two-second renewable lease, explicit
support confirmation, current model/boot IDs and at least 200 contiguous,
stationary BRAKE frames. `zero.commit` takes `CalibrationMetadata` with the
joint mapping, captured rotor reference and direction/reference confirmations.
The controller verifies every field and uses `toJointState` for feedback.
Uncalibrated `q_joint` contains NaN internally; a future JSON serializer must
emit `null`. A reference inside hard limits but outside guarded soft limits
stays `CALIBRATION_VALID` until supported movement returns inside soft limits.

`plan.validate` copies a complete `Plan` by value. Its identity includes
`plan_id`, `model_hash`, `calibration_id` and `board_boot_id`; reused IDs are
rejected. The trajectory must begin at the stopped measured pose, finish
stopped, and satisfy duration, spacing, guarded limits, speed and acceleration
checks. Runtime interpolation is cubic Hermite. Validation additionally checks
its analytic extrema, so a safe sequence of endpoints cannot conceal an
unsafe interpolated segment. `plan.execute` accepts only the stored plan ID
and identity and checks the measured start again. It does not accept a target
joint vector or a replacement trajectory.

`collision_checked` is an attestation from the trusted planner. This core has
no geometry collision checker or authentication layer; a future service must
establish the trusted planner boundary before exposing hardware execution.
Completed execution enters `GRAVITY_HOLD` for continued support. `gravity.set`
ramps the gravity scale, and local PD plus gravity/damping torque is limited
as a total by each rotor cap and torque slew. Vendor KP/KD stay zero so they
cannot bypass the total torque bounds.

`stop` and `estop` are accepted without a lease. Stop commands BRAKE output;
BRAKE releases active holding and is not a mechanical safety brake. ESTOP is
latched across stop, disconnect and reconnect. Explicit `fault.reset` requires
a lease, fresh healthy BRAKE feedback and support confirmation; it invalidates
calibration and returns to `READ_ONLY`. Hardware errors, stale/replayed
feedback, excessive speed/temperature/torque, joint limit violations, tracking
errors, expired active leases and missed control cycles latch a fault and
remove output. Physical support and a physical cutoff remain required.

`FakeController` is a deterministic ideal tracking fixture driven by
`advance(dt_s)` and optional `setRotorPosition`; it is not a dynamics simulator.
Tests cover the calibration gate, out-of-soft-limit reference, trajectory
immutability/interpolation, lease ownership/expiry, gravity ramp and torque
slew, watchdogs, malformed feedback and ESTOP reset behavior.

Build without a vendor SDK:

```sh
cmake -S . -B build
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

`-DQARM_BUILD_HARDWARE=ON` additionally builds `Qarm::ControllerHardware` with a
`MotorBusAdapter`. Constructing that adapter opens the serial device and uses
the existing process-wide bus lock. Its `step` checks the cycle/lease/feedback
watchdogs before FOC output, exchanges each joint, and accepts new feedback.
Bus exceptions latch a fault and request BRAKE; incomplete acknowledgements
are returned as a physical cutoff diagnostic. The adapter continues BRAKE
telemetry while faulted to support explicit reset. The application must call
commands and `step` on the same owning thread, and run the loop continuously;
this library creates no background loop or watchdog thread. Compilation was
verified with the local SDK; no serial device or robot was exercised.

Legacy standalone tools require `-DQARM_BUILD_MAINTENANCE_APPS=ON` and have no
installation rules. `QMINI_ARM_BUILD_APPS` remains a deprecated build alias.
