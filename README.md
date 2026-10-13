# GRS Ground Station

C++ ground control station of the GRS project: MAVLink links to the
vehicles (MAVSDK), the NMPC/LMPC controller with its NMHE estimator, the
catapult launchers, an RTK base station and a browser dashboard. It also
builds `grs_batchsim`, a closed-loop batch simulator that runs the same
controller core.

## Requirements

- Linux, CMake, a C++20 compiler. Ninja is recommended: the generated
  CasADi solvers are compiled one at a time to limit memory.
- MAVSDK v3.11.1 (only for the GCS executable):
  ```
  wget https://github.com/mavlink/MAVSDK/releases/download/v3.11.1/libmavsdk-dev_3.11.1_ubuntu24.04_amd64.deb
  sudo dpkg -i libmavsdk-dev_3.11.1_ubuntu24.04_amd64.deb
  ```
- CasADi: shipped in `casadi/`, nothing to install.
- yaml-cpp and PX4-GPSDrivers are fetched by CMake.

## Build

```
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

`-DGRS_BUILD_GCS=OFF` builds only the MAVSDK-free parts (`grs_batchsim`,
tests), for example on a compute server.

## Run

```
cd build
./GRS_GroundStation --config=inputFilesExamples/configuration.yaml
```

`dashboard/` and `inputFilesExamples/` are copied into `build/` when CMake
configures: rerun `cmake` after editing them in the repo, or edit the copies.

The YAML picks the vehicle profile (`configuration.yaml`: one UAV tethered
to the ground, `configuration_twoUav.yaml`: two UAVs and a payload), the
links (`Pixhawk`, `Catapults`) and the rates. `--help` lists the options
(`--sitl`, `--UAVs`, `--hlc-freq`, ...), `--listCommand` the console
commands. The dashboard opens at http://localhost:8080.

Typical session, typed in the console:

1. `connect`, then `startRtk auto 0` on the field.
2. `setOrigin lat lon alt` at the anchor, or "Set origin from payload GPS"
   on the dashboard. Wait for `Offset for sysId N`.
3. On the trajectory page: capture the live positions, generate, apply.
4. `startController`: the controller solves from the launcher.
5. `catapultConnect`, `catapultArm`, then `catapultFire` (`launch` in SITL).
   The launch is refused until the frame offsets, the trajectory and a
   valid plan are ready, with the reason in the log.

## Vehicle side

The vehicles run the GrsPlane ArduPilot fork:

- `CONTROL_SYSTEM_STATE` (146) carries the controller state (NED position
  and velocity from the EKF origin, airspeed, attitude). The GCS requests
  it at `stateRateHz` (40 Hz), at most 80 % of `SCHED_LOOP_RATE`.
- `SET_ATTITUDE_TARGET` carries roll, pitch, throttle, the angle-of-attack
  feedforward [rad] and flags, see `source/Definitions/grsMavlinkConventions.h`.
  It is applied in GUIDED and XNAV only.
- Set every `SRn_*` of the GCS port to -1: the vehicle then sends only what
  the GCS requests.

In SITL, give the start location with `--home lat,lon,alt,yaw` and run the
GCS with `--sitl`.

## Batch simulation

```
./grs_batchsim --config=inputFilesExamples/batchsim_twoUav.yaml --reference=generate --out=results
```

`--help` lists the options (Monte Carlo samples, controllers, noise,
delays, parallel jobs). `tools/batchsim_replay.html` replays a run in the
browser.

## Documentation

`docs/ARCHITECTURE.md` describes the process and the modules; each module
has its own file in `docs/` (Communication, Control, Dashboard, Simulation,
Trajectory, ...).
