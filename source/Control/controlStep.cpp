/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "controlStep.h"

#include "mpcController.h"
#include "nmheEstimator.h"

ControlStep::ControlStep(Controller& controller, std::unique_ptr<EstimatorRunner> runner, const int estimatorNu)
    : m_controller(controller)
    , m_runner(std::move(runner))
{
    if (m_runner) {
        m_appliedControl.assign(static_cast<size_t>(estimatorNu), 0.0);
    }
}

std::map<uint8_t, uavCommandsFlags> ControlStep::tick(const std::map<uint8_t, uavStates>& navStates) {
    m_estimateAppliedThisTick = false;

    if (m_runner) {
        // 1. Newest NMHE result finished since the last tick, if any; the
        //    controller zero-order holds the previous one otherwise.
        if (auto est = m_runner->takeEstimate()) {
            m_controller.setDisturbanceEstimate(est->wind, est->d);
            m_appliedEstimate = std::move(*est);
            m_estimateAppliedThisTick = true;
        }

        // 2. This tick's sample. Estimator::addSample() pairs each sample with
        //    the control applied going from the PREVIOUS sample to this one:
        //    the previous tick's command, not the one computed below (which
        //    only starts acting after this sample). Always raw telemetry: no
        //    pre-launch reference substitution (an MpcController-specific
        //    safeguard; static pre-launch telemetry is a fine cold-start
        //    window for the estimator).
        std::vector<double> measuredState;
        m_buildEstimatorStateVector(navStates, measuredState);
        m_runner->pushSample(measuredState, m_appliedControl);
    }

    // 3. NMPC solve.
    auto cmds = m_controller.solve(navStates);

    if (m_runner) {
        // Physical units for the applied control: thrust in Newtons, roll/pitch in radians.
        // The GCS's thrust->rpm conversion happens after tick()
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

void ControlStep::m_buildEstimatorStateVector(const std::map<uint8_t, uavStates>& states, std::vector<double>& out) const {
    // Same joint-across-vehicles layout as MpcController::m_unpackLatestStates()
    // numUavs blocks of 8, then one block of 6 for the payload if the controller has one.
    static constexpr int kUavBlockSize = 8;
    static constexpr int kPayloadBlockSize = 6;

    const int numUavs = m_controller.numUavs();
    const bool hasPayload = m_controller.hasPayload();

    out.assign(static_cast<size_t>(kUavBlockSize) * numUavs + (hasPayload ? kPayloadBlockSize : 0), 0.0);

    for (const auto& [sysId, s] : states) {
        if (sysId <= numUavs) {
            const size_t blockOffset = static_cast<size_t>(sysId - 1) * kUavBlockSize;
            out.at(blockOffset + 0) = s.northMeter;
            out.at(blockOffset + 1) = s.eastMeter;
            out.at(blockOffset + 2) = s.downMeter;
            out.at(blockOffset + 3) = s.northMeterSecond;
            out.at(blockOffset + 4) = s.eastMeterSecond;
            out.at(blockOffset + 5) = s.downMeterSecond;
            out.at(blockOffset + 6) = grs::degToRad(s.rollDegree);
            out.at(blockOffset + 7) = grs::degToRad(s.pitchDegree);
        } else if (hasPayload) {
            const size_t blockOffset = static_cast<size_t>(kUavBlockSize) * numUavs;
            out.at(blockOffset + 0) = s.northMeter;
            out.at(blockOffset + 1) = s.eastMeter;
            out.at(blockOffset + 2) = s.downMeter;
            out.at(blockOffset + 3) = s.northMeterSecond;
            out.at(blockOffset + 4) = s.eastMeterSecond;
            out.at(blockOffset + 5) = s.downMeterSecond;
        }
    }
}

ControlStack buildControlStack(YAML::Node& node, const bool withEstimator) {
    ControlStack stack;
    stack.solver = ConfigurationParser::parseSolverConfig(node);

    // Backend selection is a startup-only choice. numUavs picks the concrete SolverBackend once, here.
    stack.controller = std::make_unique<MpcController>(stack.solver, createSolverBackend(stack.solver.numUavs));

    // Presence of "EstimatorConfiguration" in the YAML is the enable.
    if (withEstimator) {
        stack.estimator = ConfigurationParser::parseEstimatorConfig(node);
        if (stack.estimator) {
            stack.estimatorInstance = std::make_unique<NmheEstimator>(
                *stack.estimator, createEstimatorBackend(stack.estimator->numUavs));
        }
    }
    return stack;
}
