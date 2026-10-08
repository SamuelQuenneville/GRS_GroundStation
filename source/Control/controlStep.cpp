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
        m_layout = {.numUavs = m_controller.numUavs(), .hasPayload = m_controller.hasPayload()};
        m_measuredState.assign(m_layout.size(), 0.0);
    }
}

std::map<uint8_t, uavCommandsFlags> ControlStep::tick(const std::map<uint8_t, uavStates>& navStates, const double time) {
    // 1. Newest NMHE estimate, if one finished since the last tick. Only in
    //    flight: the estimator only sees in-flight samples.
    if (m_runner) {
        if (auto est = m_runner->takeEstimate(); est && m_controller.inFlight()) {
            m_controller.setDisturbanceEstimate(est->wind, est->d);
            m_appliedEstimate = std::move(*est);
        }
    }

    // 2. NMPC solve, which also updates the launch phase.
    auto cmds = m_controller.solve(navStates, time);

    if (m_runner) {
        // 3. This tick's measured state with the control applied up to it
        //    (the previous tick's command), in flight only: on the launcher
        //    and in the catapult stroke the model does not hold, and the
        //    estimate would saturate. The window restarts at each change.
        const bool flying = m_controller.inFlight();
        if (flying != m_wasInFlight) {
            m_runner->reset();
            m_appliedEstimate = {};
            m_wasInFlight = flying;
        }
        if (flying) {
            grs::control::fillStateVector(navStates, m_layout, m_measuredState);
            m_runner->pushSample(m_measuredState, m_appliedControl);
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
            stack.estimatorInstance = std::make_unique<NmheEstimator>(*stack.estimator);
        }
    }
    return stack;
}
