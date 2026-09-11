# Trajectory module

Generates the reference trajectory (payload path, both aircraft paths +
takeoffs, tether forces, per-aircraft attitude/thrust commands) that the NMPC
controller tracks. Originally ported from a MATLAB pipeline
(`trajectory_generation`); that history no longer matters for using or
reading this code and is not covered here. See `docs/ARCHITECTURE.md` for
how this module fits into the rest of the system.

## Scope

Hard-coded to two tethered aircraft + one payload — generalizing to N
aircraft is out of scope. Field calibration (real origin, real heading,
live-GPS launch pose) is a separate, explicit post-processing step
(`applyFieldCalibration`, `snapToLiveLaunchPositions`) rather than part of
the core generation math — `generate()` itself always produces the same
local-NED trajectory for a given config, independent of where the vehicles
actually are in the world.

## `TrajectoryConfig` (`trajectoryConfig.h`)

Plain struct grouping every input the generator needs: `world` (wind, air
density, gravity), `aircraft` (mass, aerodynamic coefficients), `tether`
(length, mass, slip-clutch payout), `payload` (mass, drag), `payloadPath`
and `aircraftPath` (maneuver shape — climb/move distances, speeds,
accelerations, phase angles, radius), plus `fieldHeadingDeg` /
`originOffsetNed` for field calibration. All fields have working defaults —
constructing a default `TrajectoryConfig` and calling `generate()` produces
a valid trajectory.

Call `finalize()` once after setting any `tether.*` fields (or after loading
config from YAML) — it derives `tether.segmentMass`/`segmentLength`/
`segmentLinCoordNorm` from `length`/`linearMass`/`nSegments`, and resolves
`tether.lengthAtLaunch < 0` ("not set") to `tether.length` (no payout
modeled). Nothing else in the generator recomputes these, so skipping
`finalize()` after changing tether fields leaves stale derived values.

## `TrajectoryGenerator` (`trajectoryGenerator.h`/`.cpp`)

Constructed with a `TrajectoryConfig`; `generate()` runs the full pipeline
and returns a `GeneratedMission`:

1. Payload path (climb, move, hold phases).
2. Each aircraft's path around the payload, plus its takeoff phase
   (`generateAircraftTakeoff`) — the tether's slip-clutch payout, if
   configured, is modeled here as a smooth ramp of tether length from
   `lengthAtLaunch` to `length`.
3. Tether forces, solved for the two-aircraft bridle.
4. Per-aircraft attitude/thrust commands from the resulting forces.
5. Takeoff phase prepended to each aircraft's mission-phase timeline.

`generate()` is `const` and has no side effects on `m_config` — calling it
twice returns the same mission. Output is in the config's local NED frame,
not yet field-calibrated.

### `GeneratedMission`

- `time` — one timestamp per sample, shared by every array below.
- `payload` — the payload's `KinematicSample` (pos/vel/acc) timeline.
- `aircraft` — one `AircraftTimeline` per aircraft (`payloadFrame` +
  `inertial` NED), `size() == config.aircraftPath.phaseRad.size()`.
- `controls` — one array of `ControlSample` (roll/pitch/yaw/thrust/AoA/lift
  direction) per aircraft, takeoff + mission concatenated.

### Post-processing (applied by the caller, not inside `generate()`)

- **`applyFieldCalibration(mission, fieldHeadingDeg, originOffset)`** —
  yaws every position/velocity/acceleration vector about the Down axis, then
  translates by `originOffset`. Both arguments default to identity, so this
  is a no-op unless a caller opts in.
- **`snapToLiveLaunchPositions(mission, liveLaunchPositionsNed)`** — after
  field calibration, rigidly translates each aircraft's *entire* trajectory
  so its first (launch) sample lands exactly on a measured live GPS fix,
  independently per aircraft. Position-only; velocity/acceleration/attitude
  are untouched since a pure translation doesn't change them.
  `liveLaunchPositionsNed[k]` must line up with `mission.aircraft[k]`'s
  indexing, so call this before `extractSubset()`.
- **`extractSubset(mission, selection)`** — slices a `SubsetSelection` (a
  subset of aircraft indices, a sample-count cap, and an optional
  `includePayload` override) out of an already-generated mission, for
  exercising a reduced-order NMPC build without a second `generate()` run.
  The payload array is always copied through (just truncated); `includePayload`
  only tells the caller what to pass to `toSolverReference()`.
- **`toSolverReference(mission, hasPayload)`** — flattens a mission into the
  `[x0 u0 x1 u1 ... xN uN]` stride `NMPCController` expects: per stage,
  `numUavs` state blocks of 8 (N,E,D,vN,vE,vD,roll,pitch), then (if
  `hasPayload`) one state block of 6 for the payload, then `numUavs` control
  blocks of 3 (thrust,roll,pitch). Must match whatever `SolverConfiguration`
  the CasADi-generated `solver.c` was actually built with — in particular
  its `dt`, which the generator has no way to check against `config.simDt`
  (see the simplification list).

## Validating output

`tests/golden/` holds reference CSVs (position/velocity for both aircraft
and the payload, all 8 control columns per aircraft) that
`tests/validate_trajectory.cpp` diffs a fresh `generate()` run against.
Current worst-case error across every field: ~7e-12 (floating-point-noise
level).

Build and run it directly (no MAVSDK/CasADi needed — this only touches
`source/Mathematics` and `source/Trajectory`):

```sh
g++ -std=c++20 -O2 -Isource \
    source/Trajectory/trajectoryGenerator.cpp \
    source/Trajectory/tests/validate_trajectory.cpp \
    -o /tmp/validate_trajectory
/tmp/validate_trajectory source/Trajectory/tests/golden
```

Or, with the full project configured via CMake:
`cmake --build . --target trajectory_generator_tests`.

If you regenerate the goldens (e.g. after changing a config default),
`tests/generate_golden.m` (run under Octave against a checkout of the
original MATLAB `trajectory_generation` project) is what produced the
existing ones — see its header comment for the exact steps.

## Using it

```cpp
grs::trajgen::TrajectoryConfig config; // defaults produce a valid trajectory
// ... adjust config fields for this mission ...
grs::trajgen::TrajectoryGenerator generator(config);
grs::trajgen::GeneratedMission mission = generator.generate();

// Optional: rotate/translate for the real field.
// grs::trajgen::TrajectoryGenerator::applyFieldCalibration(mission, fieldHeadingDeg, originOffsetNed);

std::vector<double> reference = grs::trajgen::TrajectoryGenerator::toSolverReference(mission, /*hasPayload=*/true);
```

## Operational note — payload GPS link

`CommunicationManager` keys everything by the real MAVLink `system_id()`,
not position in a list, so the payload doesn't need special-casing there —
add it as one more `Pixhawk.endpoints` entry in the YAML config, with a
sysId higher than every UAV's (e.g. `3` for a 2-UAV setup). In SITL mode,
`gcsConfig::numUavs` also drives how many ports `connectAll()` opens, so it
needs bumping to `numUavs + 1` for a simulated payload link. Known gap:
`UavTelemetrySnapshot`'s live WebSocket push
(`GroundControlStation::m_pushDashboardSnapshot`) doesn't distinguish a
payload sysId from a UAV sysId — once the payload has its own link, it will
show up as a spurious "UAV-03"-style panel on the live telemetry dashboard
(`index.html`) until that's filtered.
