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
   - **MPC**: `ControlStep::tick()` (controller solve, then the NMHE
     sample push and, on its own cadence, the NMHE solve; see below), then
     converts each UAV's raw thrust command through `thrust2rpm()` (see
     `docs/Powertrain.md`), then fires `nmpcDebugCallback` with the
     controller's `DebugInfo`.
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

## `ControlStep` (`controlStep.h`/`.cpp`)

One MPC-mode tick, shared by `ControlInterface` and `grs_batchsim` (see
`docs/Simulation.md`) so both run identical per-tick logic: hand the
controller the newest finished NMHE estimate (if any), push this tick's
sample to the estimator, solve the NMPC. It never solves the NMHE itself.
Each estimator sample is paired with the control applied over the interval
that ends at that sample, i.e. the previous tick's command.
`buildControlStack()` builds the controller and optional estimator from a
YAML profile, the way `ControlInterface::initialize()` does.

## `EstimatorRunner` (`estimatorRunner.h`/`.cpp`)

How the NMHE runs relative to the control loop:

- **`ThreadedEstimatorRunner`** (GCS, SITL, flight): the NMHE has its own
  thread at `nmheFrequency` (wall clock) and owns the `Estimator`. The
  control thread only appends samples to a small queue and picks up the
  latest published estimate, both under short-held mutexes; it never waits
  for an NMHE solve, so a slow NMHE cannot delay a command. An NMHE solve
  longer than its period is counted as an overrun and logged, and the
  schedule restarts from now instead of bursting catch-up solves.
- **`DeferredEstimatorRunner`** (`grs_batchsim`): deterministic emulation of
  the thread. Same cadence, counted in ticks; the solve runs inline so runs
  are reproducible, but its result reaches the controller only after a
  latency (default: the next tick; `--nmhe-latency=<ms>|measured`).

## `Controller` (`controller.h`)

Pure interface `ControlInterface` actually talks to — `solve()`,
`initLaunch()`, trajectory load/save/generate, `getDebugInfo()`,
`getTrajectoryForVehicle()`. Deliberately agnostic to controller family
(NMPC, LMPC, TVLQR, ...); `MpcController` below is the only implementation
today. See `gcs-sitl-integration-plan.md` §3a for why this split exists and
what a second implementation (`TvlqrController`) would look like.

## `MpcController` (`mpcController.h`/`.cpp`)

The `Controller` implementation for the NMPC and the LMPC
(`SolverConfiguration.CONTROLLER: nmpc | lmpc`). `solve(latestStates)` fills
the solver inputs (initial state, parameter vector, warm start), runs an
`Nlpsol` (see below), unpacks the result into per-UAV commands, and moves the
reference-trajectory index forward.

The LMPC is the same problem with the dynamics and angle-of-attack
constraints linearized about the reference window: its solver is a QP with
the same bounds, constraint rows and warm start, and its parameter vector is
the NMPC's followed by `P_lin` (each stage's `Ad`, `Bd`, `cd` and alpha
rows). A second generated function computes `P_lin` from the reference
window, the wind/disturbance estimate and L0 at every solve; its time counts
in the solve time. Both are exported by `export_solver_*_lmpc.m`.

How the reference index moves is `SolverConfiguration.
REFERENCE_INDEXING`: `nearest` (default) searches forward for the reference
point closest in north/east to UAV 1; `time` advances exactly one sample per
solve, like the MATLAB sims (used by `grs_batchsim`, see
`docs/Simulation.md`). Also owns:

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

## `GeneratedFunction` / `Nlpsol` (`generatedFunction.h`, `nlpsol.h`)

`GeneratedFunction` is one CasADi-generated function from `CasadiSolver/`
(its memory slot and workspaces), picked by id (NMPC, LMPC, LMPC
linearization, NMHE) and `NUM_UAVS`. Every generated header is included only
in `generatedFunction.cpp`; adding one means one `GENERATED_API(...)` entry
in its table. `Nlpsol` adds the nlpsol buffers (8 in, 6 out, nlpsol order)
and the solution check. The parameter and bound layouts belong to the
problem, not the solver: `MpcController` and `NmheEstimator` pack them.

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
