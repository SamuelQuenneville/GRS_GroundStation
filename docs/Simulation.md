# Simulation module (`grs_batchsim`)

Closed-loop batch simulator: the GCS's own controller core (the generated
NMPC/NMHE solvers, `MpcController`, `NmheEstimator`, `ControlStep`) against
a C++ port of GRS_Controller's MATLAB truth plant, stepped on a simulated
clock as fast as the solvers allow. Built for Monte Carlo campaigns: it
reads the same `samples.csv` as the MATLAB campaign and writes the same
metrics, so the two can be compared and analyzed with the same scripts.

It is a separate executable, not a mode of `GRS_GroundStation`: no
MAVSDK, no dashboard, no wall-clock pacing. Both executables link the same
`grs_core` library, so a controller change reaches both.

```
grs_core (static lib)            Controller/Estimator + backends + generated solvers,
  |                              ControlStep, ConfigurationParser, TrajectoryGenerator, Log
  +-- GRS_GroundStation (exe)    MAVSDK, dashboard, real-time 20 Hz loop
  +-- grs_batchsim (exe)         truth plant, simulated clock, Monte Carlo runner
```

## Building

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release              # GCS + batch sim
cmake -S . -B build -DGRS_BUILD_GCS=OFF                     # batch sim only, no MAVSDK needed
cmake --build build --target grs_batchsim
```

## Running

```
grs_batchsim --config=inputFilesExamples/batchsim_twoUav.yaml \
             --reference=twoUav_reference.csv \
             --samples=<round_dir>/samples.csv --out=<round_dir> \
             --controllers=nmpc_naive,nmpc_of --jobs=8
```

Controller variants use the MATLAB campaign names: `nmpc_naive`,
`nmpc_of`, `lmpc_naive`, `lmpc_of` (`_of`: with the NMHE). The family sets
`SolverConfiguration.CONTROLLER` for the run.

`--help` lists every option. The inputs come from GRS_Controller
(`02_matlab/06_comparison/cpp_batchsim/`):

| Input | MATLAB script |
|---|---|
| reference CSV (`[x u]` per row, solver stride) | `export_reference_csv('twoUav')` |
| `samples.csv` from a campaign round's `samples.mat` | `mc_export_samples_csv(round_dir)` |
| plant golden data for `plant_model_tests` | `export_plant_golden()` |

`--samples=nominal` (default) runs one all-nominal plant (sample 0).
`--reference=generate` uses `TrajectoryGenerator`'s default mission instead
of a CSV.

`--weight-sets=nominal,track,smooth` crosses every controller with cost-weight
variants defined in the profile's `WeightSets` section (factors on
`SolverConfiguration.WEIGHT` blocks, see `batchsim_twoUav.yaml`).
`--with-nominal` adds the all-nominal plant as sample 0 to a sample file,
so each configuration's nominal run lands next to its Monte Carlo runs.

Outputs, one folder per configuration (controller, weight set), next to the
MATLAB results when `--out` is a campaign round folder:

```
<out>/cpp_nmpc_of/sample_0001.metrics.csv         one row, mc_metrics_twoUav.m names
<out>/cpp_nmpc_of/sample_0001.ts.bin              per-step signals (default; --ts-decim=k, 0 = off)
<out>/cpp_nmpc_of/sample_0001.traj.csv            only with --store-traj=k
<out>/cpp_nmpc_of__track/sample_0001.metrics.csv  weight set "track"
<out>/profile_<weight set>.yaml                   profile each weight set actually ran with
<out>/batchsim_manifest.txt                       every command line run on this folder
```

`sample_XXXX.ts.bin` holds, for every control step, the signals the metrics
are computed from: tracked-point error (payload, NED), per-UAV position
error, angle of attack, tether stretch, payload height, saturation, command
rate, solver status and time, estimation errors. GRS_Controller's
`mc_analysis/` reads it (`mc_read_timeseries.m`) to evaluate metrics per
mission phase without re-running. Format (little-endian): `char[8]
"GRSTS01\0"`, `uint32 nCols`, `uint32 nRows`, `float64 dt`, then per column
`uint32 length` + name, then `float32` data column-major. About 230 kB per
two-UAV run at full rate.

`mc_collect_results_cpp_twoUav.m` gathers them into `summary_cpp.csv`.

`tools/batchsim_replay.html` replays the stored trajectories in 3D: open
the file in a browser (it loads three.js from cdnjs, so it needs internet),
click "Open results folder" and pick the `--out` folder (or drop it on the
page). One cell per sample in a grid, every controller overlaid on the
reference ghost (add the reference CSV to the folder, any name containing
`reference`), shared camera and clock, aborted runs turn red at their abort
time. Files are read locally, nothing is uploaded.
Existing result files are skipped (resume by re-running the same command;
`--force` re-runs). All pending (sample, controller) runs go into one
queue, in sample-major order; `--jobs=N` forks N workers that each claim
the next run as soon as they are free (a shared atomic counter in
`mmap`'d memory), so no core idles while work remains and a campaign can
use up to samples x controllers cores. Each line reports the campaign
position, the worker, and an ETA; files are written atomically.
`--jobs=auto` starts one worker per physical core (recommended: the
solvers are single-threaded and compute-bound, so hyper-threads add
little), `--jobs=max` one per logical CPU. Both count only the CPUs this
process may use (affinity mask, so `taskset` and container/WSL limits
apply) and never start more workers than samples. `--cpu-info` prints the
counts and exits. Solve-time metrics from a fully loaded machine are
pessimistic; use a serial run for real-time claims.

## Components

### `ControlStep` (`Control/controlStep.h`)

One MPC-mode control tick: `Controller::solve()`, then the estimator's
sample push and, every `1/nmheFrequency`, the NMHE solve and
`setDisturbanceEstimate()`. Moved out of `ControlInterface::m_controlLoop()`
so the GCS and the batch sim run the same tick; `buildControlStack()` is the
matching shared construction from a YAML profile.

### Truth plant (`plantModel.h`)

`TwoUavPayloadPlant` and `OneGroundPlant` are line-by-line ports of
`grsTwoUavPayloadDynamicAugmented.m` and `grsOneGroundDynamicAugmented.m`
(same `ode` and `alpha` outputs), with every airframe/rig constant a
runtime parameter so a sample can perturb the plant. `integrate()` is
`doStepFineAug.m` (sub-stepped RK4/RK2/Euler, default 8 RK4 sub-steps per
control interval).

Hand-ported instead of CasADi-generated because a generated `Function`
bakes the parameters in as constants. `plant_model_tests` checks the port
against MATLAB-evaluated golden data (`export_plant_golden.m`), perturbed
parameter sets included.

### Sample mapping (`truthSample.h`)

`applySample()` is `mc_apply_sample_twoUav.m`: `rel` parameters are
factors on nominal, `abs` parameters are values, absent parameters are
nominal, unknown names are an error. The controller and NMHE never see the
sample: they keep their nominal model and the YAML `L0`.

### Runner (`closedLoopRunner.h`)

`runClosedLoop()` is `mc_run_one_twoUav.m`: fresh controller per run,
`initLaunch()` at t=0, true state starting on the reference's first sample,
the truth state (plus optional Gaussian noise) handed to the controller as
float telemetry through the same `uavStates` map the GCS builds (payload as
sysId `numUavs+1`), commands applied in physical units (no thrust to rpm
conversion), divergence aborts (non-finite state or control, tracked point
more than `--abort-err` from the reference, UAV below 1 m under ground).
Metrics are `mc_metrics_twoUav.m`'s, computed against the time-indexed
reference.

## Differences to the MATLAB loop

The controller side is the GCS's, not a copy of the MATLAB loop, which is
the point. The known differences a MATLAB/C++ comparison will show:

- **Tick timing.** MATLAB solves at every step whatever the solve time.
  `--deadline` makes the batch sim behave like the GCS loop instead: no tick
  while the previous solve still runs, the command acts when the solve ends,
  and the reference keeps its time (`skipped_frac` in the metrics).
- **Warm start.** `MpcController` shifts the last accepted primal solution
  and resets the duals to zero; MATLAB shifts the last solution, accepted or
  not, and also the duals (`shift_dual.m`). After a rejected solve, the GCS
  applies the last accepted plan's control and the LMPC keeps linearizing
  about it; MATLAB applies the rejected solution and linearizes about the
  reference.
- **Estimator timing.** The GCS runs the NMHE on its own thread; the batch
  sim emulates it deterministically (`DeferredEstimatorRunner`): the NMHE
  solves every `nmheFrequency` period and its result is used from the next
  tick on (or later, `--nmhe-latency`). MATLAB estimates first and uses the
  result in the same step, and fires when `it >= M` and
  `mod(it, nmhe_every) == 0`.
- **Command timing.** By default a command acts on the plant from the
  start of its interval, like MATLAB. `--cmd-delay=<ms>` holds the previous
  command for that long first (solve time plus link latency, as in SITL);
  `--cmd-delay=measured` uses each tick's measured NMPC solve time, which
  depends on machine load, so prefer a fixed value for campaigns run with
  `--jobs`.
- **Launch phases.** The run is launched before the first tick, from the
  reference's first state (12 m/s, above the 10 m/s `IN_FLIGHT_SPEED`
  default), so it is in flight from the first tick: measured state and NMHE
  samples from the start, as in MATLAB (see `docs/Control.md`).
- **Precision.** Telemetry and commands pass through `float`, as on the
  real link.
- **Solver statistics.** Fatrop's iteration count and return code come
  from `fatropStatus.cpp`; `ok` is return code 0 (converged), so a solve
  stopped at the iteration limit but accepted by the controller is not `ok`.
- **Noise.** `--meas-noise` uses `std::mt19937_64`, so a noisy run is
  statistically, not sample-by-sample, equivalent to MATLAB's.

`compare_cpp_matlab_run_twoUav.m` runs one sample on both sides and
reports the differences.
