/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef PARSEUTILS_H
#define PARSEUTILS_H

#pragma once

#include <charconv>
#include <optional>
#include <string>
#include <string_view>

// Small, no-throw parsing helpers shared by anything that turns user-typed
// text (CLI args, console commands) into numbers. std::stoi/std::stod/etc.
// throw on bad input, which is the wrong failure mode for text a human is
// actively typing -- one typo shouldn't be able to unwind an exception out
// of a command loop and take the whole process down. These return
// std::nullopt instead, so the caller can print a usage message and move on.
namespace grs {

    inline std::string_view trim(std::string_view s) {
        constexpr auto* whitespace = " \t\r\n";
        const size_t begin = s.find_first_not_of(whitespace);
        if (begin == std::string_view::npos) return {};
        const size_t end = s.find_last_not_of(whitespace);
        return s.substr(begin, end - begin + 1);
    }

    // Parses `text` as a fully-consumed integer of type T (e.g. int,
    // uint32_t, unsigned). Returns std::nullopt, never throws. If the
    // text is empty, isn't a valid integer, or has trailing junk after the
    // number (so "42x" is rejected rather than silently read as 42).
    template <typename T>
    std::optional<T> parseInt(std::string_view text) {
        text = trim(text);
        if (text.empty()) return std::nullopt;

        T value{};
        const auto* begin = text.data();
        const auto* end = text.data() + text.size();
        const auto [ptr, ec] = std::from_chars(begin, end, value);

        if (ec != std::errc{} || ptr != end) return std::nullopt;
        return value;
    }

    // Same idea for floating point.
    inline std::optional<double> parseDouble(std::string_view text) {
        text = trim(text);
        if (text.empty()) return std::nullopt;

        double value{};
        const auto* begin = text.data();
        const auto* end = text.data() + text.size();
        const auto [ptr, ec] = std::from_chars(begin, end, value);

        if (ec != std::errc{} || ptr != end) return std::nullopt;
        return value;
    }

}

#endif //PARSEUTILS_H
