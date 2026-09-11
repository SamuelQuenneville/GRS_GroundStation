# GRS_GroundStation — Architecture

C++ ground control station for multi-UAV formation flight with a towed
payload. This document is the system-level map: what the modules are, how
they talk to each other, and the conventions that show up in more than one
place. Module-specific design rationale lives in `docs/<Module>.md`.

## Process shape

`main.cpp` does four things, in order:

1. Loads `gcsConfig` from YAML, then applies CLI overrides
   (`--UAVs=`, `--hlc-freq=`, `--matlab=ip:port`, `--commandFile=`, `--sitl`).
2. Starts `DashboardServer` (embedded HTTP + WebSocket server, see
   `docs/Dashboard.md`) and opens a browser window pointed at it.
3. Constructs `GroundControlStation` (`gcs.h`/`gcs.cpp`), wires it to the
   dashboard, and calls `initialize(config)`.
4. Starts `ConsoleInterface` (operator command line) and blocks on a
   condition variable until told to exit, then shuts everything down in
   reverse order.

`GroundControlStation` is the composition root: it owns one instance each of
`CommunicationManager`, `ControlDispatcher`, `ControlInterface`,
`RtkBaseStation`, and `CatapultLauncher`, and wires their callbacks together.
Nothing outside `gcs.cpp` reaches into more than one of these directly.

## Module map (`source/`)

| Module | Role | Doc |
|---|---|---|
| `Control` | Control loop, NMPC solver wrapper, WGS84 → local NED frame | [`docs/Control.md`](Control.md) |
| `Communication` | MAVSDK link to each vehicle, catapult launcher protocol, RTK base station | [`docs/Communication.md`](Communication.md) |
| `Dashboard` | Embedded web server + WebSocket telemetry push | [`docs/Dashboard.md`](Dashboard.md) |
| `Trajectory` | Generates the reference trajectory the NMPC controller tracks | [`docs/Trajectory.md`](Trajectory.md) |
| `Mathematics` | Shared math utilities (`grs::radToDeg`, etc.), vector/matrix math | [`docs/Mathematics.md`](Mathematics.md) |
| `Definitions` | Shared plain-data structs (`uavStates`, `uavCommandsFlags`, `uavHealth`, solver config) used across module boundaries | [`docs/Definitions.md`](Definitions.md) |
| `Configuration` | YAML config parsing | [`docs/Configuration.md`](Configuration.md) |
| `Console` | Operator command-line interface | [`docs/Console.md`](Console.md) |
| `Log` | Text logger + structured per-tick CSV logger | [`docs/Log.md`](Log.md) |
| `Geo` | WGS84 ↔ NED coordinate conversion | [`docs/Geo.md`](Geo.md) |
| `Powertrain` | Thrust ↔ RPM conversion for the propulsion model | [`docs/Powertrain.md`](Powertrain.md) |
| `Util` | Small parsing/timing helpers used by `main.cpp` and elsewhere | [`docs/Util.md`](Util.md) |

`casadi/` is a vendored third-party SDK (CasADi's C++ headers and prebuilt
libraries), not project code — see "Vendored code" below.

## Cross-module conventions

These recur in several modules and are easy to relearn the hard way, so
they're recorded once here rather than re-explained per file.

- **State vector layout.** The NMPC solver's state vector is blocks of 8
  values per UAV (north, east, down, vN, vE, vD, roll_rad, pitch_rad),
  followed by one block of 6 values for the payload if present (no attitude
  state — it's towed, not independently attituded). `nx`/`nu` in
  `solverConfig` are joint dimensions across *all* vehicles, not per-vehicle.
  Payload presence is inferred by comparing `nx` against
  `8 × numUavs` (`NMPCController::hasPayload()`).
- **Payload sysId convention.** The payload is never a distinguished type at
  the MAVLink level — it's whichever connected sysId is *higher* than every
  UAV's sysId (`sysId > numUavs()`), highest wins if more than one. This
  convention is duplicated (deliberately, not accidentally) across
  `NMPCController::m_unpackLatestStates`, `ControlInterface::
  getLiveNavigationStates()`, `ControlInterface::getPayloadGpsFix()`, and
  `GroundControlStation::m_buildLivePositionsSnapshot()`. If you change how
  the payload is identified, all four need updating together.
- **Event propagation: `std::function` callbacks, not shared types.**
  Control-layer classes (`ControlInterface`, `NMPCController`) never include
  Dashboard headers. Instead `GroundControlStation` calls e.g.
  `ControlInterface::setNmpcDebugCallback()` in its constructor and adapts
  the `NMPCController::DebugInfo` payload into the dashboard's
  `NmpcTelemetrySnapshot` type itself. New event types should follow this
  same pattern rather than reaching into `Dashboard/` from `Control/`.
- **Snapshot-cache-and-serve.** `DashboardServer` never sends data eagerly
  from wherever it's produced; producers call `update*()` to overwrite a
  mutex-guarded snapshot, and a dedicated broadcast thread reads and pushes
  it out at a fixed rate (`broadcastRateHz`, default 5 Hz). This decouples
  telemetry arrival rate from what's sent to the browser and means a slow
  WebSocket client can't back-pressure a telemetry callback.
- **Route registration order.** New `httplib::Server::Get/Post` routes must
  be registered before `set_mount_point()` (which serves static files from
  `m_staticRoot`) — explicit routes only take precedence when registered
  first.

## Vendored code

Two directories are third-party, not project code, and are out of scope for
the comment-reduction/simplification pass:

- `casadi/` — the CasADi C++ SDK (headers, `.so`s, pkg-config files).
- `source/Dashboard/httplib.h` — [cpp-httplib](https://github.com/yhirose/cpp-httplib)
  (MIT, © Yuji Hirose), vendored as a single header. At ~21K lines, this is
  the overwhelming majority of what a raw `wc -l` on `source/Dashboard/`
  reports — the module's actual project code
  (`dashboardServer.{h,cpp}`, `dashboardTypes.h`, `jsonWriter.h`,
  `jsonReader.h`, `browserLauncher.h`) is only ~1,280 lines. See
  `docs/Dashboard.md`.

Neither should be edited, reformatted, or have its comments touched — they
should ideally not even be linted as part of this project's own code style.
