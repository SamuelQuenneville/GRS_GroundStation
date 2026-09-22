/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef ESTIMATORBACKENDFACTORY_H
#define ESTIMATORBACKENDFACTORY_H

#pragma once

#include <memory>

#include "estimatorBackend.h"

// Startup-only construction of the concrete EstimatorBackend for a run --
// same one-time-choice reasoning as createSolverBackend() (see
// solverBackendFactory.h): numUavs=1 -> OneUavNmheBackend, numUavs=2 ->
// TwoUavPayloadNmheBackend.
std::unique_ptr<EstimatorBackend> createEstimatorBackend(int numUavs);

#endif //ESTIMATORBACKENDFACTORY_H
