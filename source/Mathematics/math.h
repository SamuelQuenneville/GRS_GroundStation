/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

/**
 * @file math.h
 * @brief Core math type aliases for vectors, matrices.
 *
 * This header provides convenient typedefs for commonly used mathematical types
 * such as 2D/3D/4D vectors, 2x2/3x3/4x4 matrices.
 *
 * These aliases wrap around the templated `Vector`, `Matrix` structs.
 */

#ifndef MATH_H
#define MATH_H

#pragma once

#include "mathUtils.h"
#include "vector.h"
#include "matrix.h"

namespace grs {

/**
 * @name Vector Type Aliases
 * @brief Convenient integer, float, and double precision vector typedefs.
 * @{
 */
using Vec2i = Vector<int, 2>;   ///< 2D integer vector
using Vec3i = Vector<int, 3>;   ///< 3D integer vector
using Vec4i = Vector<int, 4>;   ///< 4D integer vector

using Vec2f = Vector<float, 2>; ///< 2D single-precision float vector
using Vec3f = Vector<float, 3>; ///< 3D single-precision float vector
using Vec4f = Vector<float, 4>; ///< 4D single-precision float vector

using Vec2d = Vector<double, 2>; ///< 2D double-precision float vector
using Vec3d = Vector<double, 3>; ///< 3D double-precision float vector
using Vec4d = Vector<double, 4>; ///< 4D double-precision float vector
/** @} */

/**
 * @name Matrix Type Aliases
 * @brief Convenient 2x2, 3x3, and 4x4 matrix typedefs.
 * @{
 */
using Matrix2i = Matrix<int, 2>;    ///< 2x2 integer matrix
using Matrix3i = Matrix<int, 3>;    ///< 3x3 integer matrix
using Matrix4i = Matrix<int, 4>;    ///< 4x4 integer matrix

using Matrix2f = Matrix<float, 2>;  ///< 2x2 single-precision matrix
using Matrix3f = Matrix<float, 3>;  ///< 3x3 single-precision matrix
using Matrix4f = Matrix<float, 4>;  ///< 4x4 single-precision matrix

using Matrix2d = Matrix<double, 2>; ///< 2x2 double-precision matrix
using Matrix3d = Matrix<double, 3>; ///< 3x3 double-precision matrix
using Matrix4d = Matrix<double, 4>; ///< 4x4 double-precision matrix
/** @} */

}

#endif //MATH_H
