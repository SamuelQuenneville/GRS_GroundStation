/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "estimatorBackendFactory.h"

#include <stdexcept>
#include <string>

#include "oneUavNmheBackend.h"
#include "twoUavPayloadNmheBackend.h"

std::unique_ptr<EstimatorBackend> createEstimatorBackend(const int numUavs) {
    switch (numUavs) {
        case 1:
            return std::make_unique<OneUavNmheBackend>();
        case 2:
            return std::make_unique<TwoUavPayloadNmheBackend>();
        default:
            throw std::runtime_error(
                "createEstimatorBackend: no EstimatorBackend implemented for numUavs=" + std::to_string(numUavs));
    }
}
