// Conversions between engine values and the provider API's plain records:
// transforms and fixed-size strings.
#pragma once

#include <ROCK/Abi.h>

#include <cstddef>
#include <cstdio>
#include <string>

namespace rock
{
    namespace physics_interaction_internal
    {
        // Target is a provider or public API transform: rotate[9], translate[3], scale.
        template <class Target>
        inline void fillProviderTransform(const RE::NiTransform& source, Target& target)
        {
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    target.rotate[static_cast<std::size_t>(row * 3 + column)] = source.rotate.entry[row][column];
                }
            }
            target.translate[0] = source.translate.x;
            target.translate[1] = source.translate.y;
            target.translate[2] = source.translate.z;
            target.scale = source.scale;
        }

        inline RE::NiTransform providerTransformToNi(const api::Transform& source)
        {
            RE::NiTransform result{};
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    result.rotate.entry[row][column] = source.rotate[static_cast<std::size_t>(row * 3 + column)];
                }
            }
            result.translate.x = source.translate[0];
            result.translate.y = source.translate[1];
            result.translate.z = source.translate[2];
            result.scale = source.scale;
            return result;
        }

        inline void copyProviderString(char* target, std::size_t targetSize, const std::string& source)
        {
            if (!target || targetSize == 0) {
                return;
            }

            std::snprintf(target, targetSize, "%s", source.c_str());
            target[targetSize - 1] = '\0';
        }
    }
}
