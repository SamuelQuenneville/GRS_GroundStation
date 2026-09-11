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

/**
 * @brief Minimal counterpart to JsonWriter -- reads one key at a time out
 * of a flat JSON object. Not a general JSON parser: only for POST bodies
 * the dashboard's own frontend sends (see docs/Dashboard.md), never for
 * arbitrary/untrusted JSON.
 */

#include <cctype>
#include <string>

class JsonReader {

public:
    explicit JsonReader(std::string body) : m_body(std::move(body)) {}

    /// @param fallback Returned if `key` is missing or not followed by a
    ///        plain number (covers missing fields, and true/false/null/
    ///        objects/arrays).
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

    /// @param fallback Returned if `key` is missing or not followed by a
    ///        literal `true`/`false` (JsonWriter::add(key, bool) always
    ///        emits one of those, never 0/1, so getNumber() can't read a
    ///        bool field back).
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
