# Dashboard module

Embedded HTTP + WebSocket server that serves the operator dashboard
(`dashboard/index.html`, `dashboard/setup3d.html`) and pushes live telemetry
to it. See `docs/ARCHITECTURE.md` for how this fits into the rest of the
system.

**Scope note:** `source/Dashboard/httplib.h` is vendored third-party code
([cpp-httplib](https://github.com/yhirose/cpp-httplib), MIT) and is not
covered by anything below — this doc and the comment-reduction pass are
about the ~1,280 lines of actual project code in this module:
`dashboardServer.{h,cpp}`, `dashboardTypes.h`, `jsonWriter.h`,
`jsonReader.h`, `browserLauncher.h`.

## `DashboardServer` (`dashboardServer.h`/`.cpp`)

Owns one `httplib::Server`, on a single port that serves both static files
and the WebSocket endpoint (`ws(s)://host:port/ws`). Binds to `localhost`
only — the dashboard has no authentication, so it isn't meant to be
reachable from other machines. (Point `listen()` at `"0.0.0.0"` instead if
LAN access is ever needed.)

Two background threads:

- **Server thread** — runs `httpServer->listen()`, which blocks for the
  server's lifetime.
- **Broadcast thread** — wakes at `broadcastRateHz` (default 5 Hz), reads
  whatever's currently in the cached snapshots, and pushes it to every
  connected WebSocket client. See "Snapshot-cache-and-serve" in
  `docs/ARCHITECTURE.md`.

Producers elsewhere in the codebase (`GroundControlStation`, mainly) call
`updateTelemetry()` / `updatePayloadTelemetry()` / `updateLauncherTelemetry()`
/ `updateNmpcTelemetry()` / `setOrigin()` / `setTrajectory()` to overwrite the
cached snapshot; none of these send anything themselves.

### REST endpoints

| Route | Purpose |
|---|---|
| `GET /api/origin` | Current `NavigationFrameManager` origin |
| `GET /api/trajectory` | Currently-loaded reference trajectory (for `setup3d.html`'s static 3D view) |
| `GET /api/trajectory/generator-defaults` | Seeds the trajectory-generator sidebar |
| `POST /api/trajectory/generate` | Pure preview — builds a trajectory but does not touch the NMPC controller |
| `POST /api/trajectory/apply` | Commits a generated trajectory to the controller |
| `GET /api/trajectory/live-positions` | On-demand read of live vehicle NED positions, for the "Capture live positions" button |
| `POST /api/origin/from-payload` | Sets the nav-frame origin from the payload's current GPS fix |
| `POST /api/trajectory/save` | Writes the currently-loaded trajectory to disk |

`generate`/`apply`/`live-positions`/`from-payload`/`save` are all backed by
handlers injected via `set*Handler()` (`std::function` members) rather than
being implemented in this file — `DashboardServer` only knows HTTP and JSON,
not trajectory generation or NMPC internals. `GroundControlStation` is what
wires the real handlers in. All of these may throw; the convention is that
an exception's `what()` becomes a 400 JSON `{"error": "..."}` body, and an
unset handler (nothing wired yet) returns 503.

### Serialization: `JsonWriter`/`JsonReader`

Deliberately minimal, hand-rolled JSON — no external JSON library. `JsonWriter`
only emits flat objects (use `addRaw()` to nest pre-serialized JSON, e.g. for
arrays — see `jsonArray()` in `dashboardTypes.h`); `JsonReader` only reads a
bare number or `true`/`false` following a given key in a flat object. This is
sufficient for the fixed, known shape of the dashboard's own telemetry
payloads and POST bodies — it is **not** a general JSON parser and should not
be reused for anything that needs to handle arbitrary/untrusted JSON.

### `dashboardTypes.h`

Plain data types mirrored 1:1 with what `dashboard/js/app.js` reads
(`setStat`/`setInfo`/`setHealth`/`setStatus` calls) — renaming a field here
means renaming it in the frontend too. Types: `UavTelemetrySnapshot`,
`PayloadTelemetrySnapshot`, `LauncherTelemetrySnapshot`,
`NmpcTelemetrySnapshot`, `OriginSnapshot`, `TrajectorySnapshot` (+
`TrajectoryVehicleSnapshot`/`TrajectoryPointJson`), `TrajectoryGenerationParams`,
`LivePositionsSnapshot` (+ `LiveVehicleFix`).

`TrajectoryGenerationParams` is the operator-adjustable subset of
`grs::trajgen::TrajectoryConfig` exposed on the dashboard's trajectory
generator sidebar. It carries the densest rationale in this module — several
fields (`uav1PhaseDeg`/`uav2PhaseDeg`, `snapToLiveLaunchPosition` +
`uavNSnap*`, `tetherLengthAtLaunchMeters`, `testEnabled` + `test*`) exist
to support specific dashboard features: real launch-position capture,
slip-clutch tether payout, and reduced-order testing — see
`docs/Trajectory.md`. The struct itself should keep only a one-line pointer
comment per field once the longer rationale is centralized there (see the
simplification flag list).

### `browserLauncher.h`

`posix_spawnp`-based launcher that opens `chromium --app=<url>` with a
trimmed environment (clears `GTK_MODULES`) so the dashboard opens
automatically in an app-style window on startup. Known gotcha (see
`docs/ARCHITECTURE.md`'s note on debugging lessons): VMware/Mesa setups can
fail to get a working WebGL context through `chromium` this way; the fix
used during development was switching the hardcoded `"chromium"` to
`"firefox"`. This is currently hardcoded, not configurable — flagged in the
simplification list.
