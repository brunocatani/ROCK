#pragma once

#include <cstdint>

namespace rock::renderer_offsets
{
    /*
     * FO4VR 1.2.72 BSLightingShader geometry setup. Blind disassembly on
     * 2026-08-23 verified that RDX is BSRenderPass*, [RDX+0x18] is the
     * BSGeometry*, and the function builds both eye transforms directly from
     * NiAVObject::world at geometry+0x70.
     */
    constexpr std::uintptr_t kFunc_BSLightingShaderSetupGeometry = 0x28B6B70;
}
