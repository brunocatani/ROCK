#pragma once
#include "Boundary.h"
namespace rock::api::boundary {
    inline void convert(rock::api::Point3& out, const rock::provider::RockProviderPoint3& in) {
        out.y = static_cast<decltype(out.y)>(in.y);
        out.z = static_cast<decltype(out.z)>(in.z);
    }
    inline void convert(rock::provider::RockProviderPoint3& out, const rock::api::Point3& in) {
        out.y = static_cast<decltype(out.y)>(in.y);
        out.z = static_cast<decltype(out.z)>(in.z);
    }
    inline void convert(rock::api::Transform& out, const rock::provider::RockProviderTransform& in) {
        for (std::size_t i=0; i<std::size(out.translate); ++i) out.translate[i] = in.translate[i];
        out.scale = static_cast<decltype(out.scale)>(in.scale);
    }
    inline void convert(rock::provider::RockProviderTransform& out, const rock::api::Transform& in) {
        for (std::size_t i=0; i<std::size(out.translate); ++i) out.translate[i] = in.translate[i];
        out.scale = static_cast<decltype(out.scale)>(in.scale);
    }
    inline void convert(rock::api::Bounds3& out, const rock::provider::RockProviderBounds3& in) {
        convert(out.max, in.max);
        out.valid = static_cast<decltype(out.valid)>(in.valid);
        out.reserved = static_cast<decltype(out.reserved)>(in.reserved);
    }
    inline void convert(rock::provider::RockProviderBounds3& out, const rock::api::Bounds3& in) {
        convert(out.max, in.max);
        out.valid = static_cast<decltype(out.valid)>(in.valid);
        out.reserved = static_cast<decltype(out.reserved)>(in.reserved);
    }
}
