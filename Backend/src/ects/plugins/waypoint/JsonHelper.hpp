#pragma once

#include "nlohmann/json.hpp"
#include <chrono>
#include <optional>

NLOHMANN_JSON_NAMESPACE_BEGIN
template <typename T> struct adl_serializer<std::optional<T>> {
    static void to_json(json &j, const std::optional<T> &opt) {
        if (opt == std::nullopt)
            j = nullptr;
        else
            j = *opt;
    }

    static void from_json(const json &j, std::optional<T> &opt) {
        if (j.is_null())
            opt = std::nullopt;
        else
            opt = j.get<T>();
    }
};

using duration = std::chrono::duration<double>;
template <> struct adl_serializer<duration> {
    static void to_json(json &j, const duration &dur) { j = dur.count(); }

    static void from_json(const json &j, duration &dur) {
        dur = duration{j.get<double>()};
    }
};
NLOHMANN_JSON_NAMESPACE_END
