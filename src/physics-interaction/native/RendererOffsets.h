#pragma once

#include <cstdint>

namespace rock::renderer_offsets
{
    /*
     * FO4VR 1.2.72 common geometry-to-eye transform worker. Four independent
     * shader setup paths call the wrapper at 0x1D14A60, which selects the eye
     * state and tail-jumps here. RCX is the source NiTransform*, RDX is the
     * transform mode, R8 is the output matrix, and R9 is the selected eye
     * state. Held geometry is identified by exact pointer equality with
     * NiAVObject::world at geometry+0x70.
     */
    constexpr std::uintptr_t kFunc_GeometryEyeTransform = 0x1D14CC0;
}
