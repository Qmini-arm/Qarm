# Control-domain protocol v1

`FakeArmBackend.command(name, payload=None, *, request_id, client_id)` returns
`schema_version`, `request_id`, `accepted`, `result`, `error`, and `snapshot`.
`snapshot()` returns a JSON-compatible dictionary; `advance(dt)` advances the
kinematic simulation clock and interpolates a validated trajectory. It never
opens hardware. `hardware=True` rejects commands until a C++ adapter exists.

The backend enforces a single lease owner, idempotent request IDs, calibration
identity (model hash, board boot ID, joints, and motor IDs), soft limits, speed
limits, start tolerance, and collision revalidation through an injected checker.
Client `collision_checked` flags do not authorize a plan. Uncalibrated
`joints[].q_joint` is `null`; zero capture creates a candidate and only an exact
`zero.commit` can make it valid. `control.enable` remains explicit afterward.

The current simulation uses piecewise-linear joint positions with derived
velocity, and gravity mode holds the pose with zero simulated torque. Active
lease expiry while controlling enters `fault`; `fault.reset` returns to
read-only and requires a new lease and explicit enable.
