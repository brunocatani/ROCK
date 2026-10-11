#pragma once
#include <cstddef>

namespace rock::loose_weapon_state {
    // NiMatrix3 has three padded NiPoint4 rows; the provider uses nine floats.
    // Copy coefficients, never the native matrix storage or its padding.
    template<class NativeRows>
    constexpr void packRotation(float (&packed)[9], const NativeRows& rows) noexcept {
        for(std::size_t r=0;r<3;++r)for(std::size_t c=0;c<3;++c)
            packed[r*3+c]=rows[r][c];
    }
    template<class NativeRows>
    constexpr void unpackRotation(NativeRows& rows, const float (&packed)[9]) noexcept {
        for(std::size_t r=0;r<3;++r)for(std::size_t c=0;c<3;++c)
            rows[r][c]=packed[r*3+c];
    }
}
