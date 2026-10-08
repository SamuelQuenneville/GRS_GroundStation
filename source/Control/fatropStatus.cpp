/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "fatropStatus.h"

#include <utility>
#include <fatrop/ocp/OCPCInterface.h>

namespace {

// Per thread: the controller and the NMHE solve on different threads.
thread_local FatropStatus t_last;

} // namespace

extern "C" fatrop_int grs_fatrop_ocp_c_solve(FatropOcpCSolver* solver) {
    const fatrop_int ret = fatrop_ocp_c_solve(solver);
    t_last = {.returnCode = ret, .iterations = fatrop_ocp_c_get_stats(solver)->iterations_count};
    return ret;
}

FatropStatus takeFatropStatus() {
    return std::exchange(t_last, {});
}
