/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 *
 */

#ifndef JSONWRITER_H
#define JSONWRITER_H

#pragma once

// Minimal helper for building small, flat-ish JSON objects without pulling in
// an external JSON library. This is only meant for *emitting* the telemetry
// payloads the dashboard sends to the browser -- it does not parse JSON.

#include <sstream>
#include <string>

class JsonWriter {

public:
    JsonWriter() { oss_ << "{"; }

    JsonWriter& add(const std::string& key, const std::string& value) {
        separator();
        oss_ << quote(key) << ":" << quote(value);
        return *this;
    }

    JsonWriter& add(const std::string& key, const char* value) {
        return add(key, std::string(value));
    }

    JsonWriter& add(const std::string& key, const double value) {
        separator();
        oss_ << quote(key) << ":" << value;
        return *this;
    }

    JsonWriter& add(const std::string& key, const int value) {
        separator();
        oss_ << quote(key) << ":" << value;
        return *this;
    }

    JsonWriter& add(const std::string& key, const bool value) {
        separator();
        oss_ << quote(key) << ":" << (value ? "true" : "false");
        return *this;
    }

    // Embeds an already-serialized JSON value (object/array/etc.) verbatim,
    // e.g. for nesting one JsonWriter's output inside another.
    JsonWriter& addRaw(const std::string& key, const std::string& rawJson) {
        separator();
        oss_ << quote(key) << ":" << rawJson;
        return *this;
    }

    std::string str() const { return oss_.str() + "}"; }

private:
    void separator() {
        if (!first_) oss_ << ",";
        first_ = false;
    }

    static std::string quote(const std::string& s) {
        std::string out = "\"";
        out.reserve(s.size() + 2);
        for (const char c : s) {
            switch (c) {
                case '"':  out += "\\\""; break;
                case '\\': out += "\\\\"; break;
                case '\n': out += "\\n";  break;
                case '\r': out += "\\r";  break;
                case '\t': out += "\\t";  break;
                default:   out += c;
            }
        }
        out += "\"";
        return out;
    }

    std::ostringstream oss_;
    bool first_ = true;
};

#endif //JSONWRITER_H
