/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

/**
 * @file matrix.h
 * @brief Generic fixed-size square matrix template implementation (2×2, 3×3, 4×4).
 *
 * This file provides a constexpr-capable matrix implementation with basic
 * linear algebra operations such as determinant, trace, inverse, transpose,
 * and matrix multiplication. It also includes utilities for constructing
 * rotation matrices and homogeneous transformations.
 */

#ifndef MATRIX_H
#define MATRIX_H

#pragma once

#include "vector.h"

namespace grs {

/**
 * @brief Generic fixed-size square matrix class.
 *
 * A statically-sized NxN matrix with elements stored in row-major order.
 * Supports sizes 2×2, 3×3, and 4×4, and requires arithmetic scalar types.
 *
 * @tparam T Arithmetic scalar type (e.g. float, double, int).
 * @tparam N Matrix dimension (2, 3, or 4).
 */
template <typename T, std::size_t N>
struct Matrix {
    static_assert(N >= 2 && N <= 4, "Matrix only supports 2x2, 3x3, 4x4");
    static_assert(std::is_arithmetic_v<T>, "Matrix scalar type must be arithmetic");

    /** @brief Internal storage of matrix elements in row-major order. */
    std::array<T, N*N> data{};

    /**
     * @brief Construct a matrix from an initializer list.
     *
     * Missing values are initialized to zero if fewer than N×N values are provided.
     *
     * @param values Elements in row-major order.
     */
    constexpr Matrix(std::initializer_list<T> values) {
        std::size_t count = std::min(values.size(), data.size());
        std::copy_n(values.begin(), count, data.begin());
        if (count < data.size()) {
            std::fill(data.begin() + count, data.end(), T{});
        }
    }

    /**
     * @brief Default constructor. Initializes all elements to zero.
     */
    constexpr Matrix() noexcept {
        data.fill(T{}); // zero initialize
    }

    /**
     * @brief Element access (mutable).
     * @param r Row index [0  N-1].
     * @param c Column index [0  N-1].
     * @return Reference to element (@p r, @p c).
     */
    constexpr T& operator()(const std::size_t r, const std::size_t c) noexcept { return data[r*N + c]; }

    /**
     * @brief Element access (const).
     * @param r Row index [0  N-1].
     * @param c Column index [0  N-1].
     * @return Const reference to element (@p r, @p c).
     */
    constexpr const T& operator()(const std::size_t r, const std::size_t c) const noexcept { return data[r*N + c]; }

    /**
     * @brief Get the matrix size.
     * @return Number of rows/columns (N).
     */
    [[nodiscard]] static constexpr std::size_t size() noexcept { return N; }

    /**
     * @brief Create a diagonal matrix.
     * @param values Diagonal values.
     * @return Matrix with values on the diagonal and zeros elsewhere.
     */
    [[nodiscard]] static constexpr Matrix diagonal(const std::array<T,N>& values) noexcept {
        Matrix R{};
        for (std::size_t i = 0; i < N; ++i) {
            R(i,i) = values[i];
        }
        return R;
    }

    /**
     * @brief Create an identity matrix.
     * @return NxN identity matrix.
     */
    [[nodiscard]] static constexpr Matrix identity() noexcept {
        Matrix R{};
        for (std::size_t i = 0; i < N; ++i) {
            R(i,i) = T(1);
        }
        return R;
    }

    /**
     * @brief Compute the determinant.
     *
     * Implemented for N = 2 and N = 3.
     *
     * @return Determinant of the matrix.
     */
    [[nodiscard]] constexpr T determinant() const noexcept {
        if constexpr (N == 2) {
            return (*this)(0,0)*(*this)(1,1) - (*this)(0,1)*(*this)(1,0);
        } else if constexpr (N == 3) {
            return
                (*this)(0,0)*((*this)(1,1)*(*this)(2,2) - (*this)(1,2)*(*this)(2,1)) -
                (*this)(0,1)*((*this)(1,0)*(*this)(2,2) - (*this)(1,2)*(*this)(2,0)) +
                (*this)(0,2)*((*this)(1,0)*(*this)(2,1) - (*this)(1,1)*(*this)(2,0));
        } else {
            static_assert(N == 2 || N == 3, "Determinant not implemented for this size");
            return T{}; // never executed
        }
    }

    /**
     * @brief Compute the matrix trace.
     * @return Sum of diagonal elements.
     */
    [[nodiscard]] constexpr T trace() const noexcept {
        T tr = T(0);
        for (std::size_t i = 0; i < N; i++) {
            tr += (*this)(i,i);
        }
        return tr;
    }

    /**
     * @brief Compute the matrix inverse.
     *
     * Implemented for N = 2, 3, and 4. Throws if the matrix is singular.
     *
     * @throws std::runtime_error If the matrix is singular.
     * @return Inverse of the matrix.
     */
    [[nodiscard]] constexpr Matrix inverse() const {
        if constexpr (N == 2) {
            T det = determinant();
            if (std::abs(det) < T(1e-9)) throw std::runtime_error("Singular matrix");
            Matrix<T,2> R{};
            R(0,0) =  (*this)(1,1) / det;
            R(0,1) = -(*this)(0,1) / det;
            R(1,0) = -(*this)(1,0) / det;
            R(1,1) =  (*this)(0,0) / det;
            return R;
        } else if constexpr (N == 3) {
            T det = determinant();
            if (std::abs(det) < T(1e-9)) throw std::runtime_error("Singular matrix");
            Matrix<T,3> R{};
            R(0,0) =  ((*this)(1,1)*(*this)(2,2) - (*this)(1,2)*(*this)(2,1)) / det;
            R(0,1) = -((*this)(0,1)*(*this)(2,2) - (*this)(0,2)*(*this)(2,1)) / det;
            R(0,2) =  ((*this)(0,1)*(*this)(1,2) - (*this)(0,2)*(*this)(1,1)) / det;

            R(1,0) = -((*this)(1,0)*(*this)(2,2) - (*this)(1,2)*(*this)(2,0)) / det;
            R(1,1) =  ((*this)(0,0)*(*this)(2,2) - (*this)(0,2)*(*this)(2,0)) / det;
            R(1,2) = -((*this)(0,0)*(*this)(1,2) - (*this)(0,2)*(*this)(1,0)) / det;

            R(2,0) =  ((*this)(1,0)*(*this)(2,1) - (*this)(1,1)*(*this)(2,0)) / det;
            R(2,1) = -((*this)(0,0)*(*this)(2,1) - (*this)(0,1)*(*this)(2,0)) / det;
            R(2,2) =  ((*this)(0,0)*(*this)(1,1) - (*this)(0,1)*(*this)(1,0)) / det;
            return R;
        } else if constexpr (N == 4) {
            Matrix<T,4> R = *this;
            std::size_t indxc[4];
            std::size_t indxr[4];
            int ipiv[4] = {0,0,0,0};

            for (int i = 0; i < 4; i++) {
                std::size_t irow;
                std::size_t icol;
                T big = T(0);
                for (std::size_t j = 0 ; j < 4 ; j++) {
                    if (ipiv[j] != 1) {
                        for (std::size_t k = 0 ; k < 4 ; k++) {
                            if (ipiv[k] == 0) {
                                T val = std::fabs(R(j,k));
                                if (val >= big) {
                                    big = val;
                                    irow = j;
                                    icol = k;
                                }
                            } else if (ipiv[k] > 1) throw std::runtime_error("Singular matrix");
                        }
                    }
                }
                ++ipiv[icol];
                if (irow != icol) {
                    for (std::size_t k = 0 ; k < 4 ; k++) {
                        std::swap(R(irow, k), R(icol, k));
                    }
                }

                indxr[i] = irow;
                indxc[i] = icol;
                if (R(icol,icol) == T(0)) throw std::runtime_error("Singular matrix");

                T pivinv = T(1) / R(icol,icol);
                R(icol,icol) = T(1);
                for (std::size_t k = 0 ; k < 4 ; k++) {
                    R(icol,k) *= pivinv;
                }
                for (std::size_t j = 0 ; j < 4 ; j++) {
                    if (j != icol) {
                        T save = R(j,icol);
                        R(j,icol) = T(0);
                        for (std::size_t k = 0 ; k < 4 ; k++) {
                            R(j,k) -= R(icol,k) * save;
                        }
                    }
                }
            }
            for (int j = 3 ; j >= 0 ; j--) {
                if(indxr[j]!=indxc[j]) {
                    for(std::size_t r = 0 ; r < 4 ; r++) {
                        std::swap(R(r, indxr[j]), R(r, indxc[j]));
                    }
                }
            }

            return R;

        } else {
            static_assert(N==2 || N==3 || N==4,"Inverse not implemented for this size");
            return Matrix{}; // never executed
        }
    }

    /**
     * @brief Compute the transpose.
     * @return Transposed matrix.
     */
    [[nodiscard]] constexpr Matrix transpose() const noexcept {
        Matrix R{};
        for (std::size_t i = 0 ; i < N ; i++) {
            for (std::size_t j = 0 ; j < N ; j++) {
                R(i,j)=(*this)(j,i);
            }
        }
        return R;
    }

    /**
     * @brief Extract the upper-left 3×3 block of a matrix.
     * @return 3×3 matrix.
     */
    [[nodiscard]] constexpr Matrix<T,3> upperLeft3x3() const noexcept {
        Matrix<T,3> R{};
        for (std::size_t i = 0; i < 3; ++i) {
            for (std::size_t j = 0; j < 3; ++j) {
                R(i,j) = (*this)(i,j);
            }
        }
        return R;
    }

};

/**
 * @brief Matrix addition.
 */
template <typename T, std::size_t N>
constexpr Matrix<T,N> operator+(const Matrix<T,N>& matrix1, const Matrix<T,N>&  matrix2) noexcept
{
    Matrix<T,N> R{};
    for (std::size_t i = 0; i < N*N; ++i) {
        R.data[i] =  matrix1.data[i] +  matrix2.data[i];
    }
    return R;
}

/**
 * @brief Matrix subtraction.
 */
template <typename T, std::size_t N>
constexpr Matrix<T,N> operator-(const Matrix<T,N>&  matrix1, const Matrix<T,N>&  matrix2) noexcept
{
    Matrix<T,N> R{};
    for (std::size_t i = 0; i < N*N; ++i) {
        R.data[i] =  matrix1.data[i] -  matrix2.data[i];
    }
    return R;
}

/**
 * @brief Division by scalar.
 */
template <typename T, std::size_t N>
[[nodiscard]] constexpr Matrix<T,N> operator/(const Matrix<T,N>& matrix, T scalar) noexcept {
    Matrix<T,N> R{};
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t j = 0; j < N; ++j) {
            R(i, j) =  matrix(i, j) / scalar;
        }
    }
    return R;
}

/**
 * @brief Scalar multiplication (scalar * matrix).
 */
template <typename T, std::size_t N>
[[nodiscard]] constexpr Matrix<T,N> operator*(T scalar, const Matrix<T,N>&  matrix) noexcept {
    Matrix<T,N> R{};
    for (std::size_t i = 0; i < N*N; ++i) {
        R.data[i] = scalar *  matrix.data[i];
    }
    return R;
}

/**
 * @brief Scalar multiplication (matrix * scalar).
 */
template <typename T, std::size_t N>
constexpr Matrix<T, N> operator*(const Matrix<T,N>& matrix, T scalar) noexcept {
    return  scalar * matrix;
}

/**
 * @brief Matrix–vector multiplication.
 */
template <typename T, std::size_t N>
[[nodiscard]] constexpr Vector<T,N> operator*(const Matrix<T,N>& matrix, const Vector<T,N>& vector) noexcept {
    Vector<T,N> R{};
    for (std::size_t i = 0; i < N; ++i) {
        T sum{};
        for (std::size_t j = 0; j < N; ++j) {
            sum += matrix(i,j) * vector[j];
        }
        R[i] = sum;
    }
    return R;
}

/**
 * @brief Multiply a 4×4 homogeneous transformation matrix with a 3D point.
 *
 * Applies affine transformation and homogeneous division.
 */
template <typename T>
[[nodiscard]] constexpr Vector<T,3> operator*(const Matrix<T,4>& matrix, const Vector<T,3>& vector) noexcept {
    // Compute transformed coordinates
    Vector<T,3> R{
         matrix(0,0)*vector[0] + matrix(0,1)*vector[1] + matrix(0,2)*vector[2] + matrix(0,3),
         matrix(1,0)*vector[0] + matrix(1,1)*vector[1] + matrix(1,2)*vector[2] + matrix(1,3),
         matrix(2,0)*vector[0] + matrix(2,1)*vector[1] + matrix(2,2)*vector[2] + matrix(2,3)
    };

    // Homogeneous w
    T w = matrix(3,0)*vector[0] + matrix(3,1)*vector[1] + matrix(3,2)*vector[2] + matrix(3,3);

    if (w != T(0) && w != T(1)) {
        R[0] /= w;
        R[1] /= w;
        R[2] /= w;
    }
    return R;
}

/**
 * @brief Matrix–matrix multiplication.
 */
template <typename T, std::size_t N>
constexpr Matrix<T,N> operator*(const Matrix<T,N>& matrix1, const Matrix<T,N>& matrix2) noexcept {
    Matrix<T,N> R{};
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t j = 0; j < N; ++j) {
            T sum{};
            for (std::size_t k = 0 ; k < N; ++k) {
                sum += matrix1(i,k) * matrix2(k,j);
            }
            R(i,j) = sum;
        }
    }
    return R;
}

/**
 * @brief Create a rotation matrix around the X-axis.
 * <pre>
 *        | 1             0            0  |
 *    R = | 0    cos(theta)    sin(theta) |
 *        | 0   -sin(theta)    cos(theta) |
 * </pre>
 * @param angleRad Rotation angle in radians.
 * @return 3×3 rotation matrix.
 */
template <typename T>
[[nodiscard]] constexpr Matrix<T,3> rotationX(T angleRad) noexcept {
    const T c = std::cos(angleRad);
    const T s = std::sin(angleRad);

    Matrix<T,3> R{};
    R(0,0) = 1; R(0,1) =  0; R(0,2) = 0;
    R(1,0) = 0; R(1,1) =  c; R(1,2) = s;
    R(2,0) = 0; R(2,1) = -s; R(2,2) = c;
    return R;
}

/**
 * @brief Create a rotation matrix around the Y-axis.
 * <pre>
 *        |  cos(theta)   0   -sin(theta) |
 *    R = |          0    1            0  |
 *        |  sin(theta)   0    cos(theta) |
 * </pre>
 * @param angleRad Rotation angle in radians.
 * @return 3×3 rotation matrix.
 */
template <typename T>
[[nodiscard]] constexpr Matrix<T,3> rotationY(T angleRad) noexcept {
    const T c = std::cos(angleRad);
    const T s = std::sin(angleRad);

    Matrix<T,3> R{};
    R(0,0) = c; R(0,1) = 0; R(0,2) = -s;
    R(1,0) = 0; R(1,1) = 1; R(1,2) =  0;
    R(2,0) = s; R(2,1) = 0; R(2,2) =  c;
    return R;
}

/**
 * @brief Create a rotation matrix around the Z-axis.
 * <pre>
 *        |  cos(theta)   sin(theta)   0 |
 *    R = | -sin(theta)   cos(theta)   0 |
 *        |          0            0    1 |
 * </pre>
 * @param angleRad Rotation angle in radians.
 * @return 3×3 rotation matrix.
 */
template <typename T>
[[nodiscard]] constexpr Matrix<T,3> rotationZ(T angleRad) noexcept {
    const T c = std::cos(angleRad);
    const T s = std::sin(angleRad);

    Matrix<T,3> R{};
    R(0,0) =  c; R(0,1) = s; R(0,2) = 0;
    R(1,0) = -s; R(1,1) = c; R(1,2) = 0;
    R(2,0) =  0; R(2,1) = 0; R(2,2) = 1;
    return R;
}

/**
 * @brief Construct a 3×3 DCM (Z-Y-X convention) from Euler angles.
 *
 * @param roll Rotation around X-axis (rad).
 * @param pitch Rotation around Y-axis (rad).
 * @param yaw Rotation around Z-axis (rad).
 * @return 3×3 direction cosine matrix.
 */
template <typename T>
[[nodiscard]] constexpr Matrix<T,3> eulerToMat(T roll, T pitch, T yaw) noexcept {
    // Roll = X rotation
    const T cr = std::cos(roll);
    const T sr = std::sin(roll);

    // Pitch = Y rotation
    const T cp = std::cos(pitch);
    const T sp = std::sin(pitch);

    // Yaw = Z rotation
    const T cy = std::cos(yaw);
    const T sy = std::sin(yaw);

    Matrix<T,3> R{};
    R(0,0) = cp*cy;
    R(0,1) = cp*sy;
    R(0,2) = -sp;

    R(1,0) = -cr*sy + sr*sp*cy;
    R(1,1) = cr*cy + sr*sp*sy;
    R(1,2) = sr*cp;

    R(2,0) = sr*sy + cr*sp*cy;
    R(2,1) = -sr*cy + cr*sp*sy;
    R(2,2) = cr*cp;
    return R;
}

/**
 * @brief Construct a 3×3 DCM (Z-Y-X convention) from Euler angles.
 *
 * @param euler Vector of Euler angles [roll, pitch, yaw] in radians.
 * @return 3×3 direction cosine matrix.
 */
template <typename T>
[[nodiscard]] constexpr Matrix<T,3> eulerToMat(Vector<T,3>& euler) noexcept {
    // Roll = X rotation
    const T cr = std::cos(euler[0]);
    const T sr = std::sin(euler[0]);

    // Pitch = Y rotation
    const T cp = std::cos(euler[1]);
    const T sp = std::sin(euler[1]);

    // Yaw = Z rotation
    const T cy = std::cos(euler[2]);
    const T sy = std::sin(euler[2]);

    Matrix<T,3> R{};
    R(0,0) = cp*cy;
    R(0,1) = cp*sy;
    R(0,2) = -sp;

    R(1,0) = -cr*sy + sr*sp*cy;
    R(1,1) = cr*cy + sr*sp*sy;
    R(1,2) = sr*cp;

    R(2,0) = sr*sy + cr*sp*cy;
    R(2,1) = -sr*cy + cr*sp*sy;
    R(2,2) = cr*cp;
    return R;
}

/**
 * @brief Create a 4×4 rotation matrix from an axis-angle representation.
 *
 * @param axis Axis of rotation (not required to be normalized).
 * @param angleRad Rotation angle in radians.
 * @return 4×4 homogeneous rotation matrix.
 */
template <typename T>
[[nodiscard]] constexpr Matrix<T,4> makeRotation(const Vector<T,3>& axis, T angleRad) noexcept {
    const T c = std::cos(angleRad);
    const T s = std::sin(angleRad);
    const T oneMinusCos = T(1) - c;

    // Normalize axis
    const Vector<T,3> v = normalize(axis);

    Matrix<T,4> R{};
    R(0,0) = c + v[0]*v[0]*oneMinusCos;
    R(0,1) = v[0]*v[1]*oneMinusCos - v[2]*s;
    R(0,2) = v[0]*v[2]*oneMinusCos + v[1]*s;
    R(0,3) = 0;


    R(1,0) = v[0]*v[1]*oneMinusCos + v[2]*s;
    R(1,1) = c + v[1]*v[1]*oneMinusCos;
    R(1,2) = v[1]*v[2]*oneMinusCos - v[0]*s;
    R(1,3) = 0;

    R(2,0) = v[0]*v[2]*oneMinusCos - v[1]*s;
    R(2,1) = v[1]*v[2]*oneMinusCos + v[0]*s;
    R(2,2) = c + v[2]*v[2]*oneMinusCos;
    R(2,3) = 0;

    R(3,0) = 0;
    R(3,1) = 0;
    R(3,2) = 0;
    R(3,3) = 1;

    return R;
}

/**
 * @brief Create a 4×4 translation matrix.
 *
 * @param vector Translation vector [x, y, z].
 * @return 4×4 homogeneous translation matrix.
 */
template <typename T>
[[nodiscard]] constexpr Matrix<T,4> makeTranslation(const Vector<T,3>& vector) noexcept {
    Matrix<T,4> R = Matrix<T,4>::identity();
    R(0,3) = vector[0];
    R(1,3) = vector[1];
    R(2,3) = vector[2];
    return R;
}

}

#endif //MATRIX_H
