# Control module

Runs the control loop and produces per-tick commands for each vehicle, in
one of three modes. See `docs/ARCHITECTURE.md` for the state-vector layout
and payload-sysId conventions this module defines and everyone else follows.

## `ControlInterface` (`controlInterface.h`/`.cpp`)

Owns the control-loop thread (`m_controlLoop()`, running at `hlcFrequency`
Hz) and, in MPC mode, a `Controller` instance (`MpcController` today). Each tick:

1. Takes the latest telemetry (`updateStates()`'s snapshot).
2. Runs it through `NavigationFrameManager` to get NED-frame states
   (`initializeOffset()` + `toNavigationFrame()` — always called, even
   before the frame is initialized; both are no-ops until then).
3. Once the nav frame is initialized, dispatches based on `controlMode`:
   - **MPC** — `MpcController::solve()`, then converts each UAV's raw
     thrust command through `thrust2rpm()` (see `docs/Powertrain.md`), then
     fires `nmpcDebugCallback` with the controller's `DebugInfo`.
   - **MATLAB** — sends states over UDP (`m_sendDataToMatlab`) and blocks
     for a command packet back (`m_receiveDataFromMatlab`) — see the
     top-level `README.md` for the wire struct layout.
   - **ATTITUDE_FILE** — replays a pre-loaded command list
     (`setCommandsList()`) at its own recorded frequency, independent of
     the control loop's `hlcFrequency`.
4. Pushes the resulting commands out via `setCommandCallback()`.

Other public surface: trajectory load/save/generate/preview (delegates to
`Controller`/`TrajectoryGenerator`, see `docs/Trajectory.md`), origin
management (delegates to `NavigationFrameManager`), and
`getPayloadGpsFix()`/`getLiveNavigationStates()` for the dashboard's
trajectory-generator sidebar (see `docs/Dashboard.md`). Control-layer code
never includes Dashboard headers — results flow out through the
`std::function` callbacks set in `GroundControlStation`'s constructor (see
"Event propagation" in `docs/ARCHITECTURE.md`).

## `Controller` (`controller.h`)

Pure interface `ControlInterface` actually talks to — `solve()`,
`initLaunch()`, trajectory load/save/generate, `getDebugInfo()`,
`getTrajectoryForVehicle()`. Deliberately agnostic to controller family
(NMPC, LMPC, TVLQR, ...); `MpcController` below is the only implementation
today. See `gcs-sitl-integration-plan.md` §3a for why this split exists and
what a second implementation (`TvlqrController`) would look like.

## `MpcController` (`mpcController.h`/`.cpp`)

The `Controller` implementation for any NLP-based family (NMPC today, LMPC
eventually), driven through a `SolverBackend` (`solverBackend.h`) rather
than a specific codegen'd solver's symbols. `solve(latestStates)` packs the
current states into the solver's input arrays (delegating the actual
parameter-vector/bounds layout to `m_backend->packParameters()`/
`packBounds()` — that knowledge lives with the backend, not here, since a
different NLP isn't guaranteed to share it), calls `m_backend->solve()`,
unpacks the result into per-UAV commands, and shifts the reference-
trajectory index forward by one step. Also owns:

- **Reference trajectory** — `loadTrajectory()`/`saveTrajectory()` (CSV) and
  `setReferenceTrajectory()` (in-process, from `TrajectoryGenerator`), all
  storing/reading `m_referenceTrajectory` in the solver's own
  `[x0 u0 x1 u1 ... xN uN]` stride.
- **Yaw unwrapping** — `m_unwrapYaw()` tracks each UAV's continuous
  (non-wrapped) yaw across solves, since the solver's cost function
  penalizes yaw discontinuities that a naive `[-π, π]` wrap would introduce.
- **Debug/telemetry readback** — `getDebugInfo()` and
  `getTrajectoryForVehicle()`, both plain structs (defined on `Controller`)
  kept independent of `Dashboard/` (see `docs/ARCHITECTURE.md`).

State-vector and payload-detection conventions (`kUavBlockSize`,
`hasPayload()`) are documented once in `docs/ARCHITECTURE.md` rather than
repeated here.

## `SolverBackend` (`solverBackend.h`) / `OneUavNmpcBackend` (`oneUavNmpcBackend.h`/`.cpp`)

Wraps one codegen'd `nlpsol` solver's C API (8-in/6-out convention) *and*
that solver's parameter-vector/bounds packing — see the header comment in
`solverBackend.h` for why both live in one interface rather than being
split further. `OneUavNmpcBackend` is the only concrete implementation
today, wrapping `solver_oneGround_nmpc_*`; only its `.cpp` includes the
generated `solver_oneGround_nmpc.h`, so that header's macros/symbols never
reach the rest of the codebase. `SolverBackendFactory::createSolverBackend(numUavs)`
does the startup-only construction. See `gcs-sitl-integration-plan.md` §3
for the symbol-collision history this design fixes.

## `ControlDispatcher` (`controlDispatcher.h`/`.cpp`)

Small queue/thread that decouples `ControlInterface` (producer of commands)
from `CommunicationManager` (consumer, sends them to the vehicles) and vice
versa for telemetry — `attachCommunicationManager()`/
`attachControllerInput()` wire the two `std::function` callbacks together.
Runs its own dispatch thread (`m_dispatchLoop()`) reading off
`m_commandQueue`.

## `NavigationFrameManager` (`navigationFrameManager.h`/`.cpp`)

Converts WGS84 GPS states into a local NED frame anchored at an operator-set
origin, using `Geo::GeodeticConverter` (see `docs/Geo.md`). Two related but
distinct states:

- `hasOrigin()` — an origin has been set (`setOrigin()`), so there's
  something to show on the setup UI.
- `isInitialized()` — additionally, `initializeOffset()` has computed a
  per-UAV frame offset from live states. Nothing is converted until this is
  true.

`initializeOffset()` is called unconditionally every control-loop tick; it's
incremental and a no-op for any sysId it's already computed an offset for,
so calling it before the frame exists is cheap and correct. All public
methods lock `m_mutex`, since `setOrigin()` can be called from the console
thread or the dashboard's HTTP handler thread while the control loop is
concurrently reading/writing state every tick.
