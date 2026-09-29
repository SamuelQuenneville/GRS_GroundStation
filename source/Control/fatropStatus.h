/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef FATROPSTATUS_H
#define FATROPSTATUS_H

#pragma once

// Outcome of a Fatrop solve. The generated nlpsol functions return 0 even
// when Fatrop does not converge, so CMake routes their call to Fatrop
// through fatropStatus.cpp, which records this.
struct FatropStatus {
    int returnCode = -1;  // 0: converged, 1: iteration limit, other: failed, -1: no solve
    int iterations = 0;
};

// Status of the last Fatrop solve on the calling thread, then resets it.
FatropStatus takeFatropStatus();

#endif //FATROPSTATUS_H
