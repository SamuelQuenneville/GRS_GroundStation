/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

// grs_batchsim -- closed-loop batch simulator built on the GCS's own
// controller core (MpcController, NmheEstimator, generated solvers,
// ControlStep), against a C++ port of the MATLAB truth plant. Runs a Monte
// Carlo sample set (samples.csv exported from GRS_Controller) for one or
// more controller variants and weight sets, and writes one metrics file
// (mc_metrics_twoUav.m names) and one binary time series per run for the
// MATLAB analysis (mc_analysis/).
//
// Resumable (existing result files are skipped) and parallel by processes:
// every pending (sample, controller, weight set) run goes into one shared queue, and
// --jobs=N forked workers each claim the next run as soon as they are free
// (--jobs=auto: one worker per physical core, --jobs=max: one per logical
// CPU; --cpu-info prints what this machine offers).
//
// See --help for the options.

#include <sched.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <new>
#include <sstream>
#include <set>
#include <thread>

#include "Configuration/configurationParser.h"
#include "Simulation/closedLoopRunner.h"
#include "Simulation/truthSample.h"
#include "Trajectory/trajectoryGenerator.h"

namespace fs = std::filesystem;
using namespace grs::sim;

namespace {

auto kUsage = R"(grs_batchsim: closed-loop batch simulation with the GCS controller core

Required:
  --config=<yaml>          GCS YAML profile (SolverConfiguration [+ EstimatorConfiguration])
  --reference=<csv>|generate
                           reference trajectory: CSV in the solver stride (one row per
                           sample, nx+nu values, same format as MpcController::loadTrajectory,
                           e.g. twoAircraftMission exported by export_reference_csv.m), or
                           'generate' for the C++ TrajectoryGenerator's default mission
  --out=<dir>              output folder (a MATLAB round folder works: results go in
                           <dir>/cpp_<controller>/sample_XXXX.metrics.csv)

Optional:
  --samples=<csv>|nominal  Monte Carlo samples (mc_export_samples_csv.m); default nominal
  --controllers=a,b        nmpc_naive | nmpc_of | lmpc_naive | lmpc_of (default nmpc_naive,nmpc_of)
  --weight-sets=a,b        cost-weight variants from the profile's WeightSets section, crossed
                           with --controllers (default nominal = SolverConfiguration.WEIGHT as is)
  --with-nominal           also run the all-nominal plant as sample 0
  --sample=<id>            run only this sample_id
  --t-end=<s>              stop after this much simulated time (default: full reference)
  --abort-err=<m>          divergence threshold on the tracked point (default 50)
  --meas-noise=<std>       Gaussian noise on every measured state (default 0)
  --seed=<n>               noise seed (default 1)
  --n-sub=<k>              truth-plant sub-steps per control interval (default 8)
  --integrator=rk4|rk2|euler  (default rk4)
  --nmhe-hz=<f>            NMHE rate (default: GcsConfiguration.nmheFrequency, else 5)
  --nmhe-every=<k>         NMHE every k control steps (overrides --nmhe-hz)
  --nmhe-latency=<ms>|measured
                           when an NMHE result reaches the controller after its solve
                           starts (emulates the GCS's NMHE thread; rounded up to whole
                           control ticks, at least 1). Default 0 = next tick
  --cmd-delay=<ms>|measured
                           delay from measurement to the command acting on the plant
                           (default 0, MATLAB behavior). 'measured' = each tick's NMPC
                           solve time; prefer a fixed value with --jobs > 1
  --deadline               real-time controller: no tick while the previous solve still
                           runs (longer solves skip ticks, the command is held). Implies
                           --cmd-delay=measured
  --store-traj=<k>         also write every k-th step to sample_XXXX.traj.csv
  --ts-decim=<k>           keep every k-th step in sample_XXXX.ts.bin (default 1, 0 = no file)
  --jobs=<n>|auto|max      worker processes (default 1). auto = one per physical core
                           (recommended), max = one per logical CPU (hyper-threads
                           included). Workers pull (sample, configuration) runs from one
                           shared queue; never more workers than pending runs.
  --cpu-info               print the CPUs this process may use, then exit
  --force                  re-run even if a result file exists
)";

struct Args {
    std::map<std::string, std::string> kv;
    [[nodiscard]] bool has(const std::string& k) const { return kv.contains(k); }
    [[nodiscard]] std::string get(const std::string& k, const std::string& def = "") const {
        const auto it = kv.find(k);
        return it == kv.end() ? def : it->second;
    }
};

Args parseArgs(const int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string s = argv[i];
        if (s == "--help" || s == "-h") {
            std::cout << kUsage;
            std::exit(0);
        }
        if (s.rfind("--", 0) != 0) throw std::runtime_error("unexpected argument " + s);
        s = s.substr(2);
        const auto eq = s.find('=');
        if (eq == std::string::npos) a.kv[s] = "1";
        else a.kv[s.substr(0, eq)] = s.substr(eq + 1);
    }
    return a;
}

std::vector<std::string> split(const std::string& s, const char sep) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, sep)) {
        if (!item.empty()) out.push_back(item);
    }
    return out;
}

std::vector<double> readReferenceCsv(const std::string& path, const size_t stride) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open reference " + path);
    std::vector<double> ref;
    std::string line;
    size_t row = 0;
    while (std::getline(f, line)) {
        if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
        const auto cells = split(line, ',');
        if (cells.size() != stride) {
            throw std::runtime_error(path + " row " + std::to_string(row) + ": " + std::to_string(cells.size()) +
                                     " values, expected nx+nu=" + std::to_string(stride));
        }
        for (const auto& c : cells) ref.push_back(std::stod(c));
        ++row;
    }
    return ref;
}

std::string sanitize(std::string s) {
    for (char& c : s) {
        if (c == ',' || c == '\n' || c == '\r' || c == '"') c = ';';
    }
    return s;
}

std::string fmt(const double v) {
    if (std::isnan(v)) return "NaN";
    if (std::isinf(v)) return v > 0 ? "Inf" : "-Inf";
    std::ostringstream o;
    o << std::setprecision(10) << v;
    return o.str();
}

// Written to a temp name then renamed, so an interrupted run never leaves a
// half-written file that a resume would mistake for a finished one.
void writeAtomic(const fs::path& path, const std::string& content) {
    const fs::path tmp = path.string() + ".tmp" + std::to_string(::getpid());
    {
        std::ofstream f(tmp, std::ios::binary);
        if (!f) throw std::runtime_error("cannot write " + tmp.string());
        f << content;
    }
    fs::rename(tmp, path);
}

std::string metricsCsv(const std::string& controller, const std::string& weightSet, const int sampleId,
                       const RunResult& r) {
    // Column order follows mc_collect_results_twoUav.m's rows: controller,
    // weight_set, sample_id, the mc_metrics_twoUav.m fields, then the run
    // bookkeeping.
    std::ostringstream h, v;
    h << "controller,weight_set,sample_id";
    v << controller << "," << weightSet << "," << sampleId;
    for (const auto& [name, value] : r.metrics) {
        if (name == "wall_s") continue;
        h << "," << name;
        v << "," << fmt(value);
    }
    h << ",abort_reason,wall_s\n";
    double wall = NAN;
    for (const auto& [name, value] : r.metrics) {
        if (name == "wall_s") wall = value;
    }
    v << "," << sanitize(r.abortReason) << "," << fmt(wall) << "\n";
    return h.str() + v.str();
}

std::string trajCsv(const RunResult& r) {
    std::ostringstream o;
    for (size_t i = 0; i < r.trajHeader.size(); ++i) o << (i ? "," : "") << r.trajHeader[i];
    o << "\n";
    for (const auto& row : r.trajRows) {
        for (size_t i = 0; i < row.size(); ++i) o << (i ? "," : "") << fmt(row[i]);
        o << "\n";
    }
    return o.str();
}

// Binary time series, read by mc_read_timeseries.m. Little-endian:
//   char[8] "GRSTS01\0", uint32 nCols, uint32 nRows, float64 dt,
//   nCols x (uint32 nameLength, name bytes), float32 data column-major.
std::string tsBinary(const RunResult& r) {
    std::string out("GRSTS01", 8);
    auto put = [&out](const auto value) { out.append(reinterpret_cast<const char*>(&value), sizeof(value)); };
    const auto nRows = static_cast<uint32_t>(r.tsColumns.empty() ? 0 : r.tsColumns.front().size());
    put(static_cast<uint32_t>(r.tsHeader.size()));
    put(nRows);
    put(r.tsDt);
    for (const auto& name : r.tsHeader) {
        put(static_cast<uint32_t>(name.size()));
        out += name;
    }
    for (const auto& col : r.tsColumns) out.append(reinterpret_cast<const char*>(col.data()), col.size() * sizeof(float));
    return out;
}

// Returns the profile with SolverConfiguration.WEIGHT scaled by the named
// entry of the profile's WeightSets section. WEIGHT is laid out as
// Q(nx) | R(nu) | Qf(nx) | Rdu(nu) | Rdu0(nu); an entry is a list of
// {block: Q|R|Qf|Rdu|Rdu0, idx: [1-based indices within the block] | all, factor: f}.
// "nominal" is the profile's own WEIGHT.
YAML::Node applyWeightSet(const YAML::Node& config, const std::string& label, const solverConfig& sc) {
    YAML::Node out = YAML::Clone(config);
    if (label == "nominal") return out;
    const YAML::Node set = config["WeightSets"][label];
    if (!set) throw std::runtime_error("weight set '" + label + "' not found in the profile's WeightSets section");

    auto w = out["SolverConfiguration"]["WEIGHT"].as<std::vector<double>>();
    const auto nx = static_cast<size_t>(sc.nx), nu = static_cast<size_t>(sc.nu);
    if (w.size() != 2 * nx + 3 * nu) {
        throw std::runtime_error("WEIGHT has " + std::to_string(w.size()) + " entries, expected Q|R|Qf|Rdu|Rdu0 = " +
                                 std::to_string(2 * nx + 3 * nu));
    }
    const std::map<std::string, std::pair<size_t, size_t>> blocks{
        {"Q", {0, nx}}, {"R", {nx, nu}}, {"Qf", {nx + nu, nx}}, {"Rdu", {2 * nx + nu, nu}}, {"Rdu0", {2 * nx + 2 * nu, nu}}};
    for (const auto& item : set) {
        const auto name = item["block"].as<std::string>();
        const auto b = blocks.find(name);
        if (b == blocks.end()) throw std::runtime_error("weight set '" + label + "': unknown block '" + name + "'");
        const auto [offset, size] = b->second;
        std::vector<size_t> idx;
        if (item["idx"].IsScalar() && item["idx"].as<std::string>() == "all") {
            for (size_t i = 1; i <= size; ++i) idx.push_back(i);
        } else {
            idx = item["idx"].as<std::vector<size_t>>();
        }
        const auto factor = item["factor"].as<double>();
        for (const size_t i : idx) {
            if (i < 1 || i > size) {
                throw std::runtime_error("weight set '" + label + "': index " + std::to_string(i) + " outside block " +
                                         name + " (1.." + std::to_string(size) + ")");
            }
            w[offset + i - 1] *= factor;
        }
    }
    out["SolverConfiguration"]["WEIGHT"] = w;
    return out;
}

// cpp_<controller> for the nominal weights, cpp_<controller>__<weight set> otherwise.
std::string runFolder(const std::string& controller, const std::string& weightSet) {
    return "cpp_" + controller + (weightSet == "nominal" ? "" : "__" + weightSet);
}

// CPUs this process is allowed to run on (respects taskset, cgroups/WSL
// limits reflected in the affinity mask), and how many distinct physical
// cores they belong to (hyper-threads share a core).
struct CpuInfo {
    int logical = 1;
    int physical = 1;
};

CpuInfo detectCpus() {
    CpuInfo info;
    cpu_set_t set;
    CPU_ZERO(&set);
    int logical = 0;
    std::set<std::pair<int, int>> cores; // (package, core)
    if (sched_getaffinity(0, sizeof(set), &set) == 0) {
        for (int c = 0; c < CPU_SETSIZE; ++c) {
            if (!CPU_ISSET(c, &set)) continue;
            ++logical;
            const std::string base = "/sys/devices/system/cpu/cpu" + std::to_string(c) + "/topology/";
            std::ifstream pkg(base + "physical_package_id"), core(base + "core_id");
            int p = -1, k = -1;
            if (pkg >> p && core >> k) cores.emplace(p, k);
        }
    }
    info.logical = logical > 0 ? logical : std::max(1u, std::thread::hardware_concurrency());
    // Some VMs hide the topology: fall back to logical CPUs.
    info.physical = cores.empty() ? info.logical : static_cast<int>(cores.size());
    return info;
}

int resolveJobs(const std::string& value, const CpuInfo& cpu) {
    if (value == "auto") return cpu.physical;
    if (value == "max") return cpu.logical;
    const int n = std::stoi(value);
    if (n < 1) throw std::runtime_error("--jobs must be >= 1, auto or max");
    return n;
}

// Shared between the forked workers (MAP_SHARED anonymous page): the queue
// cursor and the progress counters. Lock-free atomics, so safe across
// processes.
struct SharedQueue {
    std::atomic<long> next{0};
    std::atomic<long> done{0};
    std::atomic<long> failed{0};
    std::chrono::steady_clock::rep startTicks{0};
};
static_assert(std::atomic<long>::is_always_lock_free, "need lock-free atomics to share across processes");

std::string hms(const double seconds) {
    if (!std::isfinite(seconds) || seconds < 0) return "--";
    const long s = std::lround(seconds);
    std::ostringstream o;
    if (s >= 3600) o << s / 3600 << "h" << std::setw(2) << std::setfill('0') << (s % 3600) / 60 << "m";
    else if (s >= 60) o << s / 60 << "m" << std::setw(2) << std::setfill('0') << s % 60 << "s";
    else o << s << "s";
    return o.str();
}

} // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parseArgs(argc, argv);
        const CpuInfo cpu = detectCpus();
        if (args.has("cpu-info")) {
            std::cout << "logical CPUs available: " << cpu.logical << "\n"
                      << "physical cores:         " << cpu.physical << "\n"
                      << "--jobs=auto -> " << cpu.physical << " workers, --jobs=max -> " << cpu.logical << " workers\n";
            return 0;
        }
        for (const char* req : {"config", "reference", "out"}) {
            if (!args.has(req)) {
                std::cerr << "missing --" << req << "\n\n" << kUsage;
                return 2;
            }
        }


        YAML::Node config = YAML::LoadFile(args.get("config"));
        const solverConfig sc = ConfigurationParser::parseSolverConfig(config);
        const gcsConfig gc = ConfigurationParser::parseGcsConfig(config);
        const size_t stride = sc.nx + sc.nu;

        if (std::fabs(gc.hlcFrequency * sc.dt - 1.0) > 1e-9) {
            std::cerr << "warning: GcsConfiguration.hlcFrequency (" << gc.hlcFrequency << " Hz) != 1/DT ("
                      << 1.0 / sc.dt << " Hz); the batch sim steps the controller every DT.\n";
        }

        // Reference
        std::vector<double> reference;
        if (args.get("reference") == "generate") {
            grs::trajgen::TrajectoryGenerator generator{grs::trajgen::TrajectoryConfig{}};
            auto mission = generator.generate();
            if (sc.numUavs == 1) {
                // One UAV tethered to the ground: first aircraft only, no payload.
                grs::trajgen::SubsetSelection selection;
                selection.uavIndices = std::vector<size_t>{0};
                mission = grs::trajgen::TrajectoryGenerator::extractSubset(mission, selection);
            }
            reference = grs::trajgen::TrajectoryGenerator::toSolverReference(mission, sc.numUavs > 1);
            if (reference.size() % stride != 0) {
                throw std::runtime_error("generated reference does not match nx+nu of the loaded profile");
            }
        } else {
            reference = readReferenceCsv(args.get("reference"), stride);
        }

        RunOptions opts;
        if (args.has("t-end")) opts.tEnd = std::stod(args.get("t-end"));
        opts.abortErrM = std::stod(args.get("abort-err", "50"));
        opts.measNoiseStd = std::stod(args.get("meas-noise", "0"));
        opts.seed = static_cast<unsigned>(std::stoul(args.get("seed", "1")));
        opts.nSub = std::stoi(args.get("n-sub", "8"));
        opts.method = parseIntegrationMethod(args.get("integrator", "rk4"));
        opts.nmheFrequency = std::stod(args.get("nmhe-hz", std::to_string(gc.nmheFrequency)));
        if (args.has("nmhe-every")) opts.nmheFrequency = 1.0 / (sc.dt * std::stoi(args.get("nmhe-every")));
        opts.storeTrajDecim = std::stoi(args.get("store-traj", "0"));
        opts.tsDecim = std::stoi(args.get("ts-decim", "1"));
        {
            const std::string lat = args.get("nmhe-latency", "0");
            opts.nmheLatencyMs = lat == "measured" ? -1.0 : std::stod(lat);
            const std::string cd = args.get("cmd-delay", "0");
            opts.cmdDelayMeasured = cd == "measured";
            opts.cmdDelayMs = opts.cmdDelayMeasured ? 0.0 : std::stod(cd);
            if (opts.nmheLatencyMs < 0.0 && lat != "measured") throw std::runtime_error("--nmhe-latency must be >= 0 or measured");
            if (opts.cmdDelayMs < 0.0) throw std::runtime_error("--cmd-delay must be >= 0 or measured");
            opts.deadline = args.has("deadline");
            if (opts.deadline) {
                if (args.has("cmd-delay") && !opts.cmdDelayMeasured) throw std::runtime_error("--deadline implies --cmd-delay=measured");
                opts.cmdDelayMeasured = true;
                opts.cmdDelayMs = 0.0;
            }
        }

        const auto controllers = split(args.get("controllers", "nmpc_naive,nmpc_of"), ',');
        for (const auto& c : controllers) (void)parseController(c); // validate names early

        auto samples = readSamples(args.get("samples", "nominal"));
        if (args.has("sample")) {
            const int only = std::stoi(args.get("sample"));
            std::erase_if(samples, [&](const Sample& s) { return s.id != only; });
            if (samples.empty()) throw std::runtime_error("sample_id " + std::to_string(only) + " not in the samples file");
        }
        if (args.has("with-nominal") && std::ranges::none_of(samples, [](const Sample& s) { return s.id == 0; })) {
            samples.insert(samples.begin(), Sample{});
        }

        const auto weightSets = split(args.get("weight-sets", "nominal"), ',');
        std::map<std::string, YAML::Node> configs;
        for (const auto& ws : weightSets) configs[ws] = applyWeightSet(config, ws, sc);

        const fs::path outDir = args.get("out");
        for (const auto& c : controllers) {
            for (const auto& ws : weightSets) fs::create_directories(outDir / runFolder(c, ws));
        }
        const bool force = args.has("force");

        // Reproducibility: the exact profile behind each weight set, and every
        // invocation on this folder.
        for (const auto& [ws, cfg] : configs) {
            YAML::Emitter e;
            e << cfg;
            writeAtomic(outDir / ("profile_" + ws + ".yaml"), std::string(e.c_str()) + "\n");
        }
        {
            std::ofstream manifest(outDir / "batchsim_manifest.txt", std::ios::app);
            const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            manifest << std::put_time(std::localtime(&now), "%F %T") << " ";
            for (int i = 0; i < argc; ++i) manifest << (i ? " " : "") << argv[i];
            manifest << "\n";
        }

        auto runName = [](const int id) {
            char name[32];
            std::snprintf(name, sizeof(name), "sample_%04d", id);
            return std::string(name);
        };

        // Pending runs, sample-major (sample 1 for every configuration, then
        // 2, ...), same as run_mc_twoUavPayload.m: workers claim them in this
        // order, so a partial campaign stays paired across configurations.
        struct Job {
            size_t sample;
            std::string controller;
            std::string weightSet;
        };
        std::vector<Job> queue;
        long already = 0;
        for (size_t si = 0; si < samples.size(); ++si) {
            for (const auto& c : controllers) {
                for (const auto& ws : weightSets) {
                    const fs::path metricsPath = outDir / runFolder(c, ws) / (runName(samples[si].id) + ".metrics.csv");
                    if (!force && fs::exists(metricsPath)) {
                        ++already;
                        continue;
                    }
                    queue.push_back({.sample = si, .controller = c, .weightSet = ws});
                }
            }
        }
        const long total = static_cast<long>(queue.size());

        const int requested = resolveJobs(args.get("jobs", "1"), cpu);
        const int workers = static_cast<int>(std::max(1L, std::min<long>(requested, total)));
        std::cout << "grs_batchsim: " << samples.size() << " sample(s) x " << controllers.size() << " controller(s) x "
                  << weightSets.size() << " weight set(s): "
                  << total << " run(s) to do, " << already << " already present; " << workers << " worker(s) ("
                  << cpu.physical << " physical cores, " << cpu.logical << " logical CPUs)\n"
                  << std::flush;
        if (total == 0) return 0;

        void* mem = mmap(nullptr, sizeof(SharedQueue), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
        if (mem == MAP_FAILED) throw std::runtime_error("mmap failed for the shared job queue");
        auto* sh = new (mem) SharedQueue();
        sh->startTicks = std::chrono::steady_clock::now().time_since_epoch().count();

        const std::string errKey = sc.numUavs > 1 ? "pay_rms_3d" : "uav1_rms_3d";

        // One worker: claim the next pending run until the queue is empty.
        auto workerLoop = [&](const int w) {
            for (long i = sh->next.fetch_add(1); i < total; i = sh->next.fetch_add(1)) {
                const Job& job = queue[static_cast<size_t>(i)];
                const Sample& s = samples[job.sample];
                const std::string& c = job.controller;
                const std::string& ws = job.weightSet;
                const fs::path dir = outDir / runFolder(c, ws);
                const std::string name = runName(s.id);
                std::ostringstream line;
                try {
                    const TruthSpec truth = applySample(s, sc.numUavs, sc.tetherL0);
                    const RunResult r = runClosedLoop(YAML::Clone(configs.at(ws)), c, reference, truth, s.id, opts);
                    if (opts.storeTrajDecim > 0) writeAtomic(dir / (name + ".traj.csv"), trajCsv(r));
                    if (opts.tsDecim > 0) writeAtomic(dir / (name + ".ts.bin"), tsBinary(r));
                    // Written last: its presence marks the run as done for a resume.
                    writeAtomic(dir / (name + ".metrics.csv"), metricsCsv("cpp_" + c, ws, s.id, r));

                    double rms = NAN, wall = NAN, stepsDone = NAN;
                    for (const auto& [k, v] : r.metrics) {
                        if (k == errKey) rms = v;
                        if (k == "wall_s") wall = v;
                        if (k == "n_steps_done") stepsDone = v;
                    }
                    line << runFolder(c, ws).substr(4) << " sample " << s.id << ": "
                         << (r.completed ? "completed" : "ABORTED (" + r.abortReason + ")") << ", "
                         << (sc.numUavs > 1 ? "payload" : "uav") << " rms " << fmt(rms) << " m, " << fmt(stepsDone)
                         << " steps in " << fmt(wall) << " s";
                } catch (const std::exception& e) {
                    sh->failed.fetch_add(1);
                    line << runFolder(c, ws).substr(4) << " sample " << s.id << ": ERROR " << e.what();
                }
                const long k = sh->done.fetch_add(1) + 1;
                const double elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now().time_since_epoch()
                    - std::chrono::steady_clock::duration(sh->startTicks)).count();
                const double eta = elapsed / static_cast<double>(k) * static_cast<double>(total - k);
                std::ostringstream out;
                out << "[" << k << "/" << total << " w" << w << "] " << line.str() << "  (elapsed " << hms(elapsed)
                    << ", ETA " << (k < total ? hms(eta) : "0s") << ")\n";
                std::cout << out.str() << std::flush; // one write per line, so workers do not interleave mid-line
            }
        };

        if (workers == 1) {
            workerLoop(0);
        } else {
            // fork() (not exec): children inherit the parsed config, reference,
            // samples and the queue. Safe here because nothing has started a
            // thread yet (Logger is never started in grs_batchsim).
            std::vector<pid_t> pids;
            for (int w = 0; w < workers; ++w) {
                const pid_t pid = fork();
                if (pid < 0) throw std::runtime_error("fork failed");
                if (pid == 0) {
                    int rc = 0;
                    try {
                        workerLoop(w);
                    } catch (...) {
                        rc = 1;
                    }
                    std::cout.flush();
                    std::_Exit(rc);
                }
                pids.push_back(pid);
            }
            for (const pid_t pid : pids) {
                int status = 0;
                waitpid(pid, &status, 0);
                if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) sh->failed.fetch_add(1);
            }
        }

        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()
            - std::chrono::steady_clock::duration(sh->startTicks)).count();
        const long failed = sh->failed.load();
        std::cout << "grs_batchsim: done, " << sh->done.load() - failed << " run(s) written, " << failed
                  << " failed, " << already << " already present, in " << hms(elapsed) << "\n";
        munmap(mem, sizeof(SharedQueue));
        return failed == 0 ? 0 : 1;
    } catch (const std::exception& e) {
        std::cerr << "grs_batchsim: " << e.what() << "\n";
        return 1;
    }
}
