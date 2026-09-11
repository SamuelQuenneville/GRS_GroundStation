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

/// Generates the reference trajectory the NMPC controller tracks: payload
/// path, both aircraft paths and takeoffs, tether forces, per-aircraft
/// attitude/thrust commands.
///
/// Scope: two tethered aircraft plus one payload.
namespace grs::trajgen {

struct KinematicSample {
    Vec3d pos = Vec3d::zeros();
    Vec3d vel = Vec3d::zeros();
    Vec3d acc = Vec3d::zeros();
};

/// One aircraft's full timeline, takeoff phase prepended.
struct AircraftTimeline {
    std::vector<KinematicSample> payloadFrame; // relative to the payload
    std::vector<KinematicSample> inertial;     // absolute NED
};

/// Per-sample commanded attitude/thrust for one aircraft.
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

    std::vector<KinematicSample> payload;              // NED, takeoff-padded (zeros) + mission
    std::vector<AircraftTimeline> aircraft;            // size() == config.aircraftPath.phaseRad.size()
    std::vector<std::vector<ControlSample>> controls;  // controls[uavIndex][sample], takeoff + mission concatenated
};

/// Narrows an already-generated mission for exercising a reduced-order
/// NMPC build, e.g. one UAV tethered to a fixed ground anchor with no
/// payload, cut off partway through the mission, without a second
/// TrajectoryConfig or generate() run. See TrajectoryGenerator::extractSubset().
struct SubsetSelection {
    /// Which mission.aircraft[]/controls[] entries to keep, and in what
    /// order. nullopt (default) keeps all of them in original order. An
    /// explicit (possibly empty) vector is used exactly as given; an empty
    /// vector is a deliberate "zero aircraft" selection, distinct from
    /// "no override" (nullopt).
    std::optional<std::vector<size_t>> uavIndices;

    /// Overrides whether toSolverReference()'s output should include the
    /// payload block. nullopt (default) lets the caller decide, e.g.
    /// ControlInterface::generateTrajectory() defers to the loaded
    /// NMPCController's own hasPayload(). Set explicitly only to
    /// deliberately mismatch the mission's own payload data (e.g. testing
    /// a no-payload build against a mission that still has a payload).
    std::optional<bool> includePayload;

    /// Truncates every array to this many leading samples. 0 (default) is
    /// no limit (the full mission length).
    size_t maxSamples = 0;
};

class TrajectoryGenerator {
public:
    explicit TrajectoryGenerator(TrajectoryConfig config);

    /// Runs the full pipeline: generatePayloadPath, generateAircraftPath
    /// (with per-UAV generateAircraftTakeoff), tether force solve, aircraft
    /// controls, then prepends the takeoff phase. Output is in the
    /// config's local NED frame; call applyFieldCalibration() afterward
    /// for a field-adjusted copy.
    [[nodiscard]] GeneratedMission generate() const;

    /// Rotates and translates an already-generated mission in place: yaws
    /// every position/velocity/acceleration vector by
    /// config.fieldHeadingDeg about the Down axis, then adds
    /// `originOffset` (e.g. the real launch point's NED position relative
    /// to the NavigationFrameManager origin). Both arguments default to
    /// identity, so calling this with no arguments is a no-op. Kept
    /// separate from generate() so that function's output stays exactly
    /// the unadjusted MATLAB-equivalent reference, uncomplicated by
    /// calibration math.
    static void applyFieldCalibration(GeneratedMission& mission, double fieldHeadingDeg, const Vec3d& originOffset = Vec3d::zeros());

    /// Rigidly translates each UAV's entire generated trajectory (already
    /// field-calibrated, i.e. real-world NED) so its first (launch) sample
    /// lands exactly on a measured live GPS fix, instead of wherever
    /// phase/z0/tetherLengthAtLaunch would otherwise place it.
    /// Position-only; velocity/acceleration/attitude are left untouched,
    /// since a pure translation doesn't change them. Independent per UAV,
    /// so e.g. two UAVs at different launcher heights each land exactly on
    /// their own measured fix, without the averaging
    /// TrajectoryConfig::AircraftPath::z0/tetherLengthAtLaunch would need
    /// (those are single values shared by every UAV).
    /// @param liveLaunchPositionsNed Indexed the same as `mission.aircraft`
    ///        (nullopt = no correction for that UAV); call this before
    ///        extractSubset(), while that indexing still matches the full
    ///        mission. A no-op when every entry is nullopt.
    static void snapToLiveLaunchPositions(GeneratedMission& mission,
        const std::vector<std::optional<Vec3d>>& liveLaunchPositionsNed);

    /// Flattens a mission into the exact [x0 u0 x1 u1 ... xN uN] stride
    /// format NMPCController::loadTrajectory()/setReferenceTrajectory()
    /// expect: each stage is numUavs state blocks of 8
    /// (N,E,D,vN,vE,vD,roll,pitch), then, if hasPayload, one state block
    /// of 6 (N,E,D,vN,vE,vD), then numUavs control blocks of 3
    /// (thrust,roll,pitch); matches
    /// NMPCController::kUavBlockSize/kPayloadBlockSize and
    /// m_extractControls's control layout.
    [[nodiscard]] static std::vector<double> toSolverReference(const GeneratedMission& mission, bool hasPayload = true);

    /// Slices `mission` down to `selection.uavIndices` (or all aircraft,
    /// if nullopt) and the first `selection.maxSamples` samples (or all,
    /// if 0). The payload array is always copied through untouched (just
    /// truncated); `selection.includePayload` only tells a caller what to
    /// pass to toSolverReference()'s `hasPayload`.
    [[nodiscard]] static GeneratedMission extractSubset(const GeneratedMission& mission, const SubsetSelection& selection);

private:
    TrajectoryConfig m_config;
};

}

#endif //TRAJECTORYGENERATOR_H
