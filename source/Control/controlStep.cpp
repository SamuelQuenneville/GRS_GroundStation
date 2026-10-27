/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "controlStep.h"

#include <iomanip>
#include <sstream>

#include "Configuration/configurationParser.h"
#include "Log/logger.h"
#include "mpcController.h"
#include "nmheEstimator.h"

namespace {

std::ostringstream logLine(const double time) {
    std::ostringstream line;
    line << std::fixed << std::setprecision(4) << Logger::instance().nowMilliseconds() << ","
         << Logger::nowWallTimeMs() << "," << time;
    return line;
}

void logEstimator(const EstimatorRunner& runner, const char* event, const double time,
                  const EstimatorRunner::Estimate* estimate = nullptr) {
    if (!Logger::instance().verbose()) return;
    const auto stats = runner.stats();
    auto line = logLine(time);
    line << "," << event << "," << stats.solves << "," << stats.published << "," << stats.overruns << "," << stats.lastSolveMs;
    if (estimate) {
        for (const double v : estimate->wind) line << "," << v;
        for (const double v : estimate->d) line << "," << v;
    }
    Logger::instance().log(LogType::ESTIMATOR, line.str());
}

} // namespace

ControlStep::ControlStep(Controller& controller, std::unique_ptr<EstimatorRunner> runner, const int estimatorNu, const double sampleDt)
    : m_controller(controller)
    , m_runner(std::move(runner))
    , m_sampleDt(sampleDt)
{
    if (m_runner) {
        m_appliedControl.assign(static_cast<size_t>(estimatorNu), 0.0);
        m_layout = {.numUavs = m_controller.numUavs(), .hasPayload = m_controller.hasPayload()};
        m_measuredState.assign(m_layout.size(), 0.0);
    }
}

std::map<uint8_t, uavCommandsFlags> ControlStep::tick(const std::map<uint8_t, uavStates>& navStates, const double time) {
    // 1. Newest NMHE estimate, if one finished since the last tick. Only in
    //    flight: the estimator only sees in-flight samples.
    if (m_runner) {
        if (auto est = m_runner->takeEstimate()) {
            const bool apply = m_controller.inFlight();
            logEstimator(*m_runner, apply ? "applied" : "ignored", time, &*est);
            if (apply) {
                m_controller.setDisturbanceEstimate(est->wind, est->d);
                m_appliedEstimate = std::move(*est);
            }
        }
    }

    if (Logger::instance().verbose()) {
        const grs::control::StateLayout layout{.numUavs = m_controller.numUavs(), .hasPayload = m_controller.hasPayload()};
        std::vector<double> measured(layout.size(), 0.0);
        grs::control::fillStateVector(navStates, layout, measured);
        auto line = logLine(time);
        for (const double v : measured) line << "," << v;
        Logger::instance().log(LogType::MEASURED_STATES, line.str());
    }

    // 2. NMPC solve, which also updates the launch phase.
    auto cmds = m_controller.solve(navStates, time);

    if (m_runner) {
        // 3. This tick's measured state with the control applied up to it
        //    (the previous tick's command), in flight only: on the launcher
        //    and in the catapult stroke the model does not hold, and the
        //    estimate would saturate. The window restarts at each change and
        //    after a gap (stale telemetry, late tick): its samples are dt apart.
        const bool flying = m_controller.inFlight();
        const bool gap = m_lastSampleTime && time - *m_lastSampleTime > 2.0 * m_sampleDt;
        if (flying != m_wasInFlight || (flying && gap)) {
            m_runner->reset();
            m_appliedEstimate = {};
            m_wasInFlight = flying;
            logEstimator(*m_runner, "reset", time);
        }
        if (flying) {
            grs::control::fillStateVector(navStates, m_layout, m_measuredState);
            m_runner->pushSample(m_measuredState, m_appliedControl);
            m_lastSampleTime = time;
        }

        // Estimator control: thrust in N, roll/pitch in radians.
        const size_t perUavNu = m_appliedControl.size() / static_cast<size_t>(m_controller.numUavs());
        for (const auto& [sysId, cmd] : cmds) {
            const size_t offset = static_cast<size_t>(sysId - 1) * perUavNu;
            if (offset + 2 < m_appliedControl.size()) {
                m_appliedControl[offset + 0] = cmd.commands.thrust;
                m_appliedControl[offset + 1] = grs::degToRad(cmd.commands.rollDegree);
                m_appliedControl[offset + 2] = grs::degToRad(cmd.commands.pitchDegree);
            }
        }
        m_runner->endTick();
    }

    return cmds;
}

ControlStack buildControlStack(YAML::Node& node, const bool withEstimator) {
    ControlStack stack;
    stack.solver = ConfigurationParser::parseSolverConfig(node);

    stack.controller = std::make_unique<MpcController>(stack.solver);

    // Presence of "EstimatorConfiguration" in the YAML is the enable.
    if (withEstimator) {
        stack.estimator = ConfigurationParser::parseEstimatorConfig(node);
        if (stack.estimator) {
            // The NMHE feeds the controller its states, wind and disturbances.
            const estimatorConfig& e = *stack.estimator;
            const solverConfig& s = stack.solver;
            if (e.nx != s.nx || e.nu != s.nu || e.np != s.np || e.nd != s.nd || e.nL0 != s.nL0
                || e.numUavs != s.numUavs || std::abs(e.dt - s.dt) > 1e-9) {
                throw std::runtime_error("EstimatorConfiguration does not match SolverConfiguration (nx, nu, np, nd, nL0, numUavs or dt)");
            }
            stack.estimatorInstance = std::make_unique<NmheEstimator>(*stack.estimator);
        }
    }
    return stack;
}
