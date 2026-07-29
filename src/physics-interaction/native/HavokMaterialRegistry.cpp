#include "physics-interaction/native/HavokMaterialRegistry.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/HavokOffsets.h"

#include <cstdint>
#include <cstring>

namespace rock::havok_material_registry
{
    namespace
    {
        constexpr std::uintptr_t kHknpWorldMaterialLibraryOffset = 0x5C8;
        constexpr std::uint16_t kInvalidMaterialId = 0xFFFF;

        void* worldMaterialLibrary(RE::hknpWorld* world)
        {
            if (!world) {
                return nullptr;
            }

            auto* materialLibrarySlot = reinterpret_cast<void**>(reinterpret_cast<char*>(world) + kHknpWorldMaterialLibraryOffset);
            return materialLibrarySlot ? *materialLibrarySlot : nullptr;
        }
    }

    RE::hknpMaterialId registerGeneratedBodyMaterial(RE::hknpWorld* world)
    {
        /*
         * ROCK-generated hand/body colliders need one shared world material.
         * Keeping the material-library layout read and native constructor calls
         * in this helper prevents domain collider code from owning Havok ABI
         * details while preserving the exact existing material configuration.
         */
        static void* cachedMaterialLibrary = nullptr;
        static RE::hknpMaterialId cachedId{ kInvalidMaterialId };

        auto* materialLibrary = worldMaterialLibrary(world);
        if (!materialLibrary) {
            ROCK_LOG_WARN(Hand, "Generated collider material library is null; using material 0");
            return { 0 };
        }
        if (cachedMaterialLibrary == materialLibrary && cachedId.value != kInvalidMaterialId) {
            return cachedId;
        }

        alignas(16) char materialBuffer[0x50];
        std::memset(materialBuffer, 0, sizeof(materialBuffer));

        using MaterialCtor_t = void (*)(void*);
        static REL::Relocation<MaterialCtor_t> materialCtor{ REL::Offset(offsets::kFunc_MaterialCtor) };
        materialCtor(materialBuffer);

        *reinterpret_cast<std::uint8_t*>(materialBuffer + 0x11) = 200;
        *reinterpret_cast<std::uint16_t*>(materialBuffer + 0x12) = 0x3C00;
        *reinterpret_cast<std::uint16_t*>(materialBuffer + 0x28) = 0x0000;
        *reinterpret_cast<std::uint8_t*>(materialBuffer + 0x18) = 2;
        *reinterpret_cast<std::uint8_t*>(materialBuffer + 0x10) = 0;

        using AddMaterial_t = void (*)(void*, std::uint16_t*, void*);
        static REL::Relocation<AddMaterial_t> addMaterial{ REL::Offset(offsets::kFunc_MaterialLibrary_AddMaterial) };

        std::uint16_t newId = kInvalidMaterialId;
        addMaterial(materialLibrary, &newId, materialBuffer);
        if (newId != kInvalidMaterialId) {
            cachedMaterialLibrary = materialLibrary;
            cachedId = { newId };
            ROCK_LOG_INFO(Hand, "Registered generated collider material id={}", newId);
        } else {
            ROCK_LOG_WARN(Hand, "Generated collider material registration failed; using material 0");
        }

        return cachedId.value != kInvalidMaterialId ? cachedId : RE::hknpMaterialId{ 0 };
    }
}
