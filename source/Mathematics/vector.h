/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

/**
 * @file vector.h
 * @brief Generic fixed-size mathematical vector implementation.
 *
 * This file defines a templated Vector class with basic linear algebra
 * operations such as addition, subtraction, dot product, cross product,
 * normalization, and linear interpolation. The implementation is designed
 * for numerical computations in the GRS Simulation Environment.
 *
 */

#ifndef VECTOR_H
#define VECTOR_H

#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <ostream>
#include <type_traits>

namespace grs {

/**
 * @brief Fixed-size mathematical vector.
 *
 * A statically allocated vector of dimension @p N with scalar type @p T.
 * Provides constructors for common 2D, 3D, and 4D cases, along with
 * a range of element-wise arithmetic operators and utility functions.
 *
 * @tparam T Arithmetic type of the vector elements (e.g., float, double, int).
 * @tparam N Dimension of the vector (must be > 0).
 */
template <typename T, std::size_t N>
struct Vector {
    static_assert(N > 0, "Vector dimension must be > 0");
    static_assert(std::is_arithmetic_v<T>, "Vector scalar type must be arithmetic");

    /** @brief Internal storage of vector elements. */
    std::array<T, N> data{};

    /**
     * @brief Construct a vector from an initializer list.
     * @param values List of values to initialize the vector.
     * @note If fewer than @p N values are provided, the remaining entries are zero-initialized.
     */
    constexpr Vector(std::initializer_list<T> values) {
        std::size_t count = std::min(values.size(), data.size());
        std::copy_n(values.begin(), count, data.begin());
        if (count < data.size()) {
            std::fill(data.begin() + count, data.end(), T{});
        }
    }

    /**
     * @brief Conversion constructor from another vector with different scalar type.
     * @tparam U Source vector scalar type.
     * @param vector Source vector to convert.
     */
    template <typename U>
    constexpr explicit Vector(const Vector<U, N>& vector) {
        for (std::size_t i = 0; i < N; ++i) {
            data[i] = static_cast<T>(vector[i]);
        }
    }

    /**
     * @brief 2D vector constructor.
     * @param x X component.
     * @param y Y component.
     * @warning Only valid for `N == 2`.
     */
    constexpr Vector(T x, T y) {
        static_assert(N == 2, "This constructor is only valid for Vector<T,2>");
        data[0] = x;
        data[1] = y;
    }

    /**
     * @brief 3D vector constructor.
     * @param x X component.
     * @param y Y component.
     * @param z Z component.
     * @warning Only valid for `N == 3`.
     */
    constexpr Vector(T x, T y, T z) {
        static_assert(N == 3, "This constructor is only valid for Vector<T,3>");
        data[0] = x;
        data[1] = y;
        data[2] = z;
    }

    /**
     * @brief 4D vector constructor.
     * @param x X component.
     * @param y Y component.
     * @param z Z component.
     * @param w W component.
     * @warning Only valid for `N == 4`.
     */
    constexpr Vector(T x, T y, T z, T w) {
        static_assert(N == 4, "This constructor is only valid for Vector<T,4>");
        data[0] = x;
        data[1] = y;
        data[2] = z;
        data[3] = w;
    }

    /**
     * @brief Element access (mutable).
     * @param i Index of the element.
     * @return Reference to the element at index @p i.
     */
    constexpr T& operator[](std::size_t i) noexcept { return data[i]; }

    /**
     * @brief Element access (const).
     * @param i Index of the element.
     * @return Const reference to the element at index @p i.
     */
    constexpr const T& operator[](std::size_t i) const noexcept { return data[i]; }

    /**
     * @brief Create a zero-initialized vector.
     * @return Vector with all elements set to zero.
     */
    [[nodiscard]] static constexpr Vector zeros() noexcept {
        Vector R{};
        for (std::size_t i = 0; i < N; ++i) {
            R[i] = T(0);
        }
        return R;
    }

    /**
     * @brief Add another vector in place.
     * @param vector Vector to add.
     * @return Reference to this vector after addition.
     */
    constexpr Vector& operator+=(const Vector& vector) noexcept {
        for (std::size_t i = 0; i < N; ++i) {
            data[i] += vector[i];
        }
        return *this;
    }

    /**
     * @brief Subtract another vector in place.
     * @param vector Vector to subtract.
     * @return Reference to this vector after subtraction.
     */
    constexpr Vector& operator-=(const Vector& vector) noexcept {
        for (std::size_t i = 0; i < N; ++i) {
            data[i] -= vector[i];
        }
        return *this;
    }

    /**
     * @brief Get the number of elements in the vector.
     * @return Vector dimension @p N.
     */
    [[nodiscard]] constexpr std::size_t size() noexcept { return N; }

    /**
     * @brief Compute the Euclidean norm (length) of the vector.
     * @return Norm of the vector.
     */
    [[nodiscard]] T norm() const noexcept {
        T sum{};
        for (std::size_t i = 0; i < N; ++i) {
            sum += data[i] * data[i];
        }
        return std::sqrt(sum);
    }

    /**
     * @brief Convert to an OpenGL reference frame.
     *
     * Reorders the vector components to match OpenGL conventions.
     * For 3D vectors, applies (x, y, z) → (x, -z, y).
     *
     * @return Converted vector.
     * @warning Only implemented for `N == 3`.
     */
    [[nodiscard]] constexpr Vector toOpenglRef() const noexcept {
        if constexpr (N == 3) {
            return Vector<T,3>{(*this)[0], -(*this)[2], (*this)[1]};
        } else {
            static_assert(N == 3, "Not implemented for this size");
            return Vector<T,3>{}; // never executed
        }
    }
};

/**
 * @brief Equality comparison.
 */
template <typename T, std::size_t N>
constexpr bool operator==(const Vector<T,N>& vector1, const Vector<T,N>& vector2) noexcept {
    for (std::size_t i = 0; i < N; ++i) {
        if (vector1[i] != vector2[i]) return false;
    }
    return true;
}

/**
 * @brief Vector addition.
 */
template <typename T, std::size_t N>
constexpr Vector<T,N> operator+(const Vector<T,N>& vector1, const Vector<T,N>& vector2) noexcept {
    Vector<T,N> r{};
    for (std::size_t i = 0; i < N; ++i) {
        r[i] = vector1[i] + vector2[i];
    }
    return r;
}

/**
 * @brief Vector subtraction.
 */
template <typename T, std::size_t N>
[[nodiscard]] constexpr Vector<T,N> operator-(const Vector<T,N>& vector1, const Vector<T,N>& vector2) noexcept {
    Vector<T,N> result{};
    for (std::size_t i = 0; i < N; ++i) {
        result[i] = vector1[i] - vector2[i];
    }
    return result;
}

/**
 * @brief Unary negation of a vector.
 */
template <typename T, std::size_t N>
[[nodiscard]] constexpr Vector<T,N> operator-(const Vector<T,N>& v) noexcept {
    Vector<T,N> result{};
    for (std::size_t i = 0; i < N; ++i) {
        result[i] = -v[i];
    }
    return result;
}

/**
 * @brief Scalar multiplication (vector * scalar).
 */
template <typename T, std::size_t N>
constexpr Vector<T,N> operator*(const Vector<T,N>& vector, T scalar) noexcept {
    Vector<T,N> R{};
    for (std::size_t i = 0; i < N; ++i) {
        R[i] = vector[i] * scalar;
    }
    return R;
}

/**
 * @brief Scalar multiplication (scalar * vector).
 */
template <typename T, std::size_t N>
constexpr Vector<T,N> operator*(T scalar, const Vector<T,N>& vector) noexcept {
    return vector * scalar;
}

/**
 * @brief Dot product of two vectors.
 */
template <typename T, std::size_t N>
[[nodiscard]] constexpr T dot(const Vector<T,N>& vector1, const Vector<T,N>& vector2) noexcept {
    T sum{};
    for (std::size_t i = 0; i < N; ++i) {
        sum += vector1[i] * vector2[i];
    }
    return sum;
}

/**
 * @brief Normalize a vector.
 * @param vector Input vector.
 * @return Normalized vector with unit length, or the original vector if its norm is zero.
 */
template <typename T, std::size_t N>
[[nodiscard]] Vector<T,N> normalize(const Vector<T,N>& vector) noexcept {
    T n = vector.norm();
    return (n > T(0)) ? (vector * (1/n)) : vector;
}

template <typename T, std::size_t N>
[[nodiscard]] constexpr Vector<T,N> cross(const Vector<T,N>&, const Vector<T,N>&) = delete;

/**
 * @brief Cross product (only available for 3D vectors).
 */
template <typename T>
[[nodiscard]] constexpr Vector<T,3> cross(const Vector<T,3>& vector1, const Vector<T,3>& vector2) noexcept {
    return Vector<T,3>{
        vector1[1]*vector2[2] - vector1[2]*vector2[1],
        vector1[2]*vector2[0] - vector1[0]*vector2[2],
        vector1[0]*vector2[1] - vector1[1]*vector2[0]
    };
}

/**
 * @brief Linear interpolation between two vectors.
 * @param vector1 Start vector.
 * @param vector2 End vector.
 * @param t Interpolation parameter (0 → vector1, 1 → vector2).
 * @return Interpolated vector.
 */
template <typename T, std::size_t N>
[[nodiscard]] constexpr Vector<T,N> lerp(const Vector<T,N>& vector1, const Vector<T,N>& vector2, T t) noexcept {
    Vector<T,N> R{};
    for (std::size_t i = 0; i < N; ++i) {
        R[i] = vector1[i] + (vector2[i] - vector1[i]) * t;
    }
    return R;
}

/**
 * @brief Cast a vector to another scalar type.
 * @tparam T Destination scalar type.
 * @tparam U Source scalar type.
 * @param v Input vector.
 * @return New vector with cast elements.
 */
template <typename T, std::size_t N, typename U>
Vector<T,N> cast(const Vector<U,N>& v) {
    return Vector<T,N>(v);
}

/**
 * @brief Stream output operator for vectors.
 * @param os Output stream.
 * @param v Vector to print.
 * @return Reference to the output stream.
 *
 * @note Prints vectors in the form `[x, y, z]`.
 */
template <typename T, std::size_t N>
std::ostream& operator<<(std::ostream& os, const Vector<T,N>& v) {
    os << '[';
    for (std::size_t i = 0; i < N; ++i) {
        os << v[i];
        if (i + 1 < N) os << ", ";
    }
    os << ']';
    return os;
}

}

#endif //VECTOR_H
