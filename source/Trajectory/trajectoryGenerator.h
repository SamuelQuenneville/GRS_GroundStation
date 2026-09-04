/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef TRAJECTORYGENERATOR_H
#define TRAJECTORYGENERATOR_H

#pragma once

#include <optional>
#include <vector>

#include "Mathematics/math.h"
#include "trajectoryConfig.h"

// C++ port of trajectory_generation's core/+traj and core/+dyn (MATLAB).
// Phase 0 of ADR-001: reproduces the existing MATLAB math sample-for-sample
// (validated against Octave-generated golden CSVs in tests/golden/) so the
// GCS no longer needs a MATLAB license to produce a reference trajectory.
// Field-calibration (real origin/heading, live GPS start pose) is
// deliberately NOT applied inside this class -- see applyFieldCalibration()
// and ADR-001 Phase 2.
//
// Scope, matching the MATLAB source exactly: two tethered aircraft + one
// payload. dyn::solveTetherForcesTwoAircraft is hard-coded for a two-aircraft
// bridle in the MATLAB project too -- generalizing to N aircraft is a
// separate piece of work, not a C++-port concern.
namespace grs::trajgen {

struct KinematicSample {
    Vec3d pos = Vec3d::zeros();
    Vec3d vel = Vec3d::zeros();
    Vec3d acc = Vec3d::zeros();
};

// One aircraft's full timeline (matches MATLAB's aircraftPath{k} after
// core/+traj/appendTakeoff.m has prepended the takeoff phase).
struct AircraftTimeline {
    std::vector<KinematicSample> payloadFrame; // relative to the payload
    std::vector<KinematicSample> inertial;     // absolute NED
};

// Per-sample commanded attitude/thrust for one aircraft -- mirrors the
// 8-column `controls` array returned by dyn.getAircraftControls:
// [roll, pitch, yaw, thrust, angleOfAttack, liftDir(3)].
struct ControlSample {
    double rollRad = 0.0;
    double pitchRad = 0.0;
    double yawRad = 0.0;
    double thrustNewton = 0.0;
    double angleOfAttackRad = 0.0;
    Vec3d liftDirection = Vec3d::zeros();
};

struct GeneratedMission {
    std::vector<double> time; // [s], one entry per sample, shared across payload/aircraft/controls

    std::vector<KinematicSample> payload;             // NED, takeoff-padded (zeros) + mission
    std::vector<AircraftTimeline> aircraft;            // size() == config.aircraftPath.phaseRad.size()
    std::vector<std::vector<ControlSample>> controls;  // controls[uavIndex][sample], takeoff + mission concatenated
};

// ADR-001 Phase 4: narrows an already-generated mission down for exercising a
// reduced-order NMPC build -- e.g. one UAV tethered to a fixed ground anchor
// (no payload), cut off partway through the mission (through the first
// loiter, say) -- without a second TrajectoryConfig or a second generate()
// run. See TrajectoryGenerator::extractSubset().
struct SubsetSelection {
    // Which mission.aircraft[]/controls[] entries to keep, and in what
    // order. nullopt (default) = keep all of them, in their original order
    // -- i.e. no UAV subsetting, matching behavior from before this existed.
    // An explicit (possibly empty) vector is used exactly as given, even if
    // empty -- that's a deliberate "zero aircraft" selection, distinct from
    // "no override".
    std::optional<std::vector<size_t>> uavIndices;

    // Overrides whether toSolverReference()'s output should include the
    // payload block. nullopt (default) = let the caller decide -- e.g.
    // ControlInterface::generateTrajectory() defers to the loaded
    // NMPCController's own hasPayload() when this is nullopt, exactly like
    // before this existed. Set explicitly only to deliberately mismatch the
    // mission's own payload data (e.g. testing a no-payload build against a
    // mission that still has a payload in it).
    std::optional<bool> includePayload;

    // Truncates every array to this many leading samples. 0 (default) = no
    // limit (the full mission length).
    size_t maxSamples = 0;
};

class TrajectoryGenerator {
public:
    explicit TrajectoryGenerator(TrajectoryConfig config);

    // Runs the full pipeline in the same order as apps/main.m:
    // generatePayloadPath -> generateAircraftPath (+ per-UAV
    // generateAircraftTakeoff) -> getTethersForces -> getAircraftControls ->
    // appendTakeoff. Output is in the config's local NED frame -- call
    // applyFieldCalibration() afterward for a field-adjusted copy.
    [[nodiscard]] GeneratedMission generate() const;

    // Rotates + translates an already-generated mission in place: yaws every
    // position/velocity/acceleration vector by config.fieldHeadingDeg about
    // the Down axis, then adds `originOffset` (e.g. the real launch point's
    // NED position relative to the NavigationFrameManager origin, once one
    // exists). Both default to identity, so calling this with no arguments
    // on a Phase-0 mission is a no-op. ADR-001 Phase 2 hook -- deliberately
    // kept separate from generate() so Phase 0 output is exactly the MATLAB
    // reference, uncomplicated by calibration math.
    static void applyFieldCalibration(GeneratedMission& mission, double fieldHeadingDeg, const Vec3d& originOffset = Vec3d::zeros());

    // Flattens a mission into the exact [x0 u0 x1 u1 ... xN uN] stride format
    // NMPCController::loadTrajectory()/setReferenceTrajectory() expect: each
    // stage is numUavs blocks of 8 states (N,E,D,vN,vE,vD,roll,pitch)
    // followed by, if hasPayload, one block of 6 (N,E,D,vN,vE,vD), then
    // numUavs blocks of 3 controls (thrust,roll,pitch) -- matching
    // NMPCController::kUavBlockSize/kPayloadBlockSize and m_extractControls's
    // control layout.
    [[nodiscard]] static std::vector<double> toSolverReference(const GeneratedMission& mission, bool hasPayload = true);

    // Slices `mission` down to `selection.uavIndices` (or all aircraft, if
    // nullopt) and the first `selection.maxSamples` samples (or all of them,
    // if 0). The payload array is always copied through untouched (just
    // truncated to the same sample count) regardless of
    // `selection.includePayload` -- that field only tells a *caller* what to
    // pass to toSolverReference()'s `hasPayload`, since an unused-but-
    // correctly-shaped payload array is harmless there.
    [[nodiscard]] static GeneratedMission extractSubset(const GeneratedMission& mission, const SubsetSelection& selection);

private:
    TrajectoryConfig m_config;
};

}

#endif //TRAJECTORYGENERATOR_H
