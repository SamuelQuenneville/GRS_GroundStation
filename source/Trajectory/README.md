# Trajectory (ADR-001)

C++ port of the [`trajectory_generation`](https://github.com/SamuelQuenneville/trajectory_generation)
MATLAB project's core math: payload path, aircraft path/takeoff, tether
forces, inverse dynamics ("Move Trajectory Generation into the GCS, Behind
the 3D Map View").

**Phase 0** (math port, no workflow change) is done -- see Scope/Validating
below. **Phase 1** (in-process wiring) is also done: `ControlInterface::
generateTrajectory()` / `GroundControlStation::generateTrajectory()` build a
trajectory with `TrajectoryGenerator` and load it directly into
`NMPCController` via the new `setReferenceTrajectory()`, no CSV round-trip.
It fires the same `setTrajectoryLoadedCallback()` `loadTrajectory(file)`
does, so the existing `setup3d.html` / `GET /api/trajectory` view picks it
up with no dashboard changes. Try it from the console with `genTraj` (uses
`TrajectoryConfig`'s defaults -- no field calibration yet, that's Phase 2).

**Phase 2** (field-calibration UI) is also done: `setup3d.html` has a
GENERATOR sidebar card exposing the operator-adjustable subset of
`TrajectoryConfig` as `TrajectoryGenerationParams` (see `dashboardTypes.h`),
including `fieldHeadingDeg` and the NED `originOffsetNed` triple. "Generate
preview" hits `POST /api/trajectory/generate` -- pure computation via
`ControlInterface::previewTrajectory()`, doesn't touch the NMPC controller --
and draws the result with the same `buildVehicleVisuals()` the read-only view
already used. "Apply to controller" hits `POST /api/trajectory/apply`, which
calls `GroundControlStation::generateTrajectory()` (same path as `genTraj`)
and then re-reads `GET /api/trajectory` so the view always reflects what's
actually loaded, not just what was last previewed.

## Scope

- Ported: `core/+traj/*` (payload path, aircraft path, aircraft takeoff,
  S-curve profiles) and `core/+dyn/*` (tether force solve, inverse dynamics,
  lift/drag model).
- **Not** ported here: field calibration (real origin, real heading, live-GPS
  start pose). That's `TrajectoryGenerator::applyFieldCalibration()`, a
  separate post-processing step kept deliberately out of the core math so
  Phase 0 output is byte-for-byte comparable to MATLAB's. Wiring it into the
  dashboard is ADR-001 Phase 2.
- Hard-coded to two tethered aircraft + one payload, same as the MATLAB
  source (`dyn::solveTetherForcesTwoAircraft` assumes a two-aircraft bridle
  there too). Generalizing to N aircraft is separate work, not a C++-port
  concern.

## Validating against MATLAB

`tests/golden/` holds CSVs produced by running the MATLAB pipeline under
Octave with the project's default `config.m`, via `tests/generate_golden.m`
-- drop that script into the root of a `trajectory_generation` checkout
(next to `apps/`, `config/`, `core/`) and run `octave generate_golden.m`
there. `tests/validate_trajectory.cpp` runs `TrajectoryGenerator` with the
equivalent default `TrajectoryConfig` and diffs every output field --
takeoff and mission position/velocity for both aircraft, all 8 columns of
each aircraft's control history, and the payload's position/velocity --
against those goldens.

Build and run it directly (no MAVSDK/CasADi needed -- this only touches
`source/Mathematics` and `source/Trajectory`):

```sh
g++ -std=c++20 -O2 -Isource \
    source/Trajectory/trajectoryGenerator.cpp \
    source/Trajectory/tests/validate_trajectory.cpp \
    -o /tmp/validate_trajectory
/tmp/validate_trajectory source/Trajectory/tests/golden
```

Or, once the full project is configured with CMake: `cmake --build . --target trajectory_generator_tests`.

Current result: worst absolute error ~7e-12 across every compared field
(floating-point-noise level) -- the port matches MATLAB, not just "close to."

If you regenerate the goldens (e.g. after changing `config.m` defaults),
rerun `tests/generate_golden.m` under Octave (see above) and copy the
resulting `/tmp/oct_*.csv` files into `tests/golden/`, then update
`TrajectoryConfig`'s defaults in `trajectoryConfig.h` to match `config.m` if
they diverged.

## Using it

```cpp
grs::trajgen::TrajectoryConfig config; // defaults mirror config.m
// ... adjust config fields for this mission ...
grs::trajgen::TrajectoryGenerator generator(config);
grs::trajgen::GeneratedMission mission = generator.generate();

// Optional, Phase 2: rotate/translate for the real field.
// grs::trajgen::TrajectoryGenerator::applyFieldCalibration(mission, fieldHeadingDeg, originOffsetNed);

std::vector<double> reference = grs::trajgen::TrajectoryGenerator::toSolverReference(mission, /*hasPayload=*/true);
// `reference` is in the exact [x0 u0 x1 u1 ... xN uN] stride NMPCController expects.
```

`toSolverReference()`'s layout must match whatever `SolverConfiguration` the
CasADi-generated `solver.c` was actually built with (`nx`/`nu`/`numUavs`,
and critically the `dt` baked in at codegen time, which isn't tracked
anywhere in the YAML config -- see ADR-001 action item #1). Feeding it a
trajectory sampled at the wrong `dt` will silently misalign the reference.
