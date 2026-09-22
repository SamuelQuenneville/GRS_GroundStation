/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef SOLVERBACKENDFACTORY_H
#define SOLVERBACKENDFACTORY_H

#pragma once

#include <memory>

#include "solverBackend.h"

// Startup-only construction of the concrete SolverBackend for a run --
// backend selection is a one-time choice made when the process starts
// (from SolverConfiguration.NUM_UAVS in the YAML), never a live switch.
// See "Backend selection is at startup, not at runtime" in
// gcs-sitl-integration-plan.md.
//
// numUavs=1 -> OneUavNmpcBackend (Phase 2), numUavs=2 ->
// TwoUavPayloadNmpcBackend (Phase 3). Nothing in ControlInterface/
// MpcController needs to change to add either -- that's the point of
// routing construction through this single factory.
std::unique_ptr<SolverBackend> createSolverBackend(int numUavs);

#endif //SOLVERBACKENDFACTORY_H
