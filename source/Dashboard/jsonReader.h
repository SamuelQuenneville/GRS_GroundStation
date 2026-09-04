/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef JSONREADER_H
#define JSONREADER_H

#pragma once

// Minimal counterpart to jsonWriter.h -- this codebase deliberately doesn't
// pull in a JSON library (see jsonWriter.h's comment), so this only reads
// what it needs to: a bare number following "key": in a flat JSON object.
// Not a general JSON parser -- fine for POST bodies our own dashboard
// frontend sends (see TrajectoryGenerationParams::fromJson in
// dashboardTypes.h), not for parsing arbitrary/untrusted JSON.

#include <cctype>
#include <string>

class JsonReader {

public:
    explicit JsonReader(std::string body) : m_body(std::move(body)) {}

    // Returns `fallback` if `key` isn't present or isn't followed by a plain
    // number (covers missing fields, and true/false/null/objects/arrays).
    [[nodiscard]] double getNumber(const std::string& key, const double fallback) const {
        const std::string needle = "\"" + key + "\"";
        size_t pos = m_body.find(needle);
        if (pos == std::string::npos) return fallback;

        pos = m_body.find(':', pos + needle.size());
        if (pos == std::string::npos) return fallback;
        ++pos;

        while (pos < m_body.size() && std::isspace(static_cast<unsigned char>(m_body[pos]))) ++pos;

        size_t end = pos;
        while (end < m_body.size() && isNumberChar(m_body[end])) ++end;
        if (end == pos) return fallback;

        try {
            return std::stod(m_body.substr(pos, end - pos));
        } catch (...) {
            return fallback;
        }
    }

    // Returns `fallback` if `key` isn't present or isn't followed by a
    // literal `true`/`false` -- JsonWriter::add(key, bool) is what emits
    // those (see jsonWriter.h), not 0/1, so getNumber() can't read them back.
    [[nodiscard]] bool getBool(const std::string& key, const bool fallback) const {
        const std::string needle = "\"" + key + "\"";
        size_t pos = m_body.find(needle);
        if (pos == std::string::npos) return fallback;

        pos = m_body.find(':', pos + needle.size());
        if (pos == std::string::npos) return fallback;
        ++pos;

        while (pos < m_body.size() && std::isspace(static_cast<unsigned char>(m_body[pos]))) ++pos;

        if (m_body.compare(pos, 4, "true") == 0) return true;
        if (m_body.compare(pos, 5, "false") == 0) return false;
        return fallback;
    }

private:
    static bool isNumberChar(const char c) {
        return std::isdigit(static_cast<unsigned char>(c)) || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E';
    }

    std::string m_body;
};

#endif //JSONREADER_H
