#include "physics-interaction/native/NativeNiNodeFactory.h"

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"

#include <array>
#include <cstring>
#include <fmt/ranges.h>

namespace rock::native_scene
{
    namespace
    {
        using NiNodeConstructor = RE::NiNode* (*)(void*, std::uint16_t);

        [[nodiscard]] bool addressIsInGameText(const std::uintptr_t address) noexcept
        {
            const auto text = REL::Module::get().segment(REL::Segment::text);
            return address >= text.address() && address < text.address() + text.size();
        }

        template <std::size_t N>
        [[nodiscard]] bool validateNativeEntry(
            const char* label,
            const std::uintptr_t offset,
            const std::array<std::uint8_t, N>& expected) noexcept
        {
            const auto address = REL::Offset(offset).address();
            std::array<std::uint8_t, N> actual{};
            if (!addressIsInGameText(address) ||
                !native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(address), actual.data(), actual.size()) ||
                actual != expected) {
                ROCK_LOG_ERROR(Init, "Native NiNode factory validation failed for {} at 0x{:X}: inGameText={} bytes={}",
                    label, address, addressIsInGameText(address), fmt::join(actual, ","));
                return false;
            }
            return true;
        }

        [[nodiscard]] NiNodeConstructor resolveNiNodeConstructor() noexcept
        {
            static const NiNodeConstructor constructor = []() noexcept -> NiNodeConstructor {
                if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) {
                    ROCK_LOG_ERROR(Init, "Native NiNode factory requires the verified Fallout4VR.exe 1.2.72 layout");
                    return nullptr;
                }

                /*
                 * The constructor/layout are specific to FO4VR 1.2.72. The
                 * allocator implementation is not: Buffout replaces Allocate,
                 * Deallocate and initialization together. RE::aligned_alloc
                 * and the native destructor must use those live engine entry
                 * points, exactly as the rest of the engine does. Requiring
                 * their original bytes rejects a valid replacement heap.
                 */
                const bool entriesMatch = validateNativeEntry(
                        "NiNode constructor",
                        offsets::kFunc_NiNode_Ctor,
                        std::array<std::uint8_t, 10>{ 0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x10 });
                if (!entriesMatch) {
                    return nullptr;
                }

                return reinterpret_cast<NiNodeConstructor>(REL::Offset(offsets::kFunc_NiNode_Ctor).address());
            }();
            return constructor;
        }
    }

    RE::NiPointer<RE::NiNode> createEngineNiNode(const std::uint16_t childCapacity) noexcept
    {
        static_assert(sizeof(RE::NiNode) == offsets::kNiNodeSize);
        static_assert(alignof(RE::NiNode) == offsets::kNiNodeAlignment);

        const auto constructor = resolveNiNodeConstructor();
        if (!constructor) {
            return nullptr;
        }

        void* const storage = RE::aligned_alloc(alignof(RE::NiNode), sizeof(RE::NiNode));
        if (!storage) {
            ROCK_LOG_ERROR(NativeScene, "Native NiNode allocation failed for 0x{:X} bytes", sizeof(RE::NiNode));
            return nullptr;
        }
        std::memset(storage, 0, sizeof(RE::NiNode));

        RE::NiNode* const node = constructor(storage, childCapacity);
        if (!node) {
            ROCK_LOG_ERROR(NativeScene, "FO4VR NiNode constructor returned null for storage {:p}", storage);
            RE::aligned_free(storage);
            return nullptr;
        }

        return RE::NiPointer<RE::NiNode>{ node };
    }
}
