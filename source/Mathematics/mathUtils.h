/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef MATHUTILS_H
#define MATHUTILS_H

#pragma once
#include <cmath>

namespace grs {
    /**
     * @brief Convert an angle in degrees to radians.
     *
     * @tparam T Floating-point type (float, double, long double).
     * @param degrees Angle in degrees.
     * @return Angle in radians.
     */
    template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
    [[nodiscard]] constexpr T degToRad(T degrees) noexcept {
        return degrees * static_cast<T>(M_PI / 180.0);
    }

    /**
     * @brief Convert an angle in radians to degrees.
     *
     * @tparam T Floating-point type (float, double, long double).
     * @param radians Angle in radians.
     * @return Angle in degrees.
     */
    template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
    [[nodiscard]] constexpr T radToDeg(T radians) noexcept {
        return radians * static_cast<T>(180.0 / M_PI);
    }
}

#endif //MATHUTILS_H
