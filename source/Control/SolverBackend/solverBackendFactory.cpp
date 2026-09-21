/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "solverBackendFactory.h"

#include <stdexcept>
#include <string>

#include "oneUavNmpcBackend.h"
// Phase 3: #include "TwoUavPayloadNmpcBackend.h"

std::unique_ptr<SolverBackend> createSolverBackend(const int numUavs) {
    switch (numUavs) {
        case 1:
            return std::make_unique<OneUavNmpcBackend>();
        // case 2:
        //     return std::make_unique<TwoUavPayloadNmpcBackend>(); // Phase 3
        default:
            throw std::runtime_error(
                "createSolverBackend: no SolverBackend implemented for numUavs=" + std::to_string(numUavs));
    }
}
