/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef LOGSCHEMA_H
#define LOGSCHEMA_H

#pragma once

#include "Definitions/controllerStructures.h"

// Column names and units of the controller CSVs (states, controls,
// measured_states, estimator, plan, timing), built from the solver layout.
// A change to what one of those rows holds must change its columns here and
// bump kLogSchemaVersion (logDefinitions.h).
namespace grs::logschema {

void registerControlColumns(const solverConfig& solver, bool hasPayload);

} // namespace grs::logschema

#endif //LOGSCHEMA_H
