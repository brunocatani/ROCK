#include "physics-interaction/native/NativeNiNodeFactory.h"

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"

#include <array>
#include <cstring>

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
                ROCK_LOG_ERROR(Init, "Native NiNode factory validation failed for {} at 0x{:X}", label, address);
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
                 * Blind-verified against Fallout4VR.exe 1.2.72 on 2026-07-20.
                 * The allocator entries are part of the constructor's child-
                 * array allocation chain and are validated with it so the
                 * complete construct/destroy ownership domain fails closed.
                 */
                const bool entriesMatch = validateNativeEntry(
                                              "MemoryManager::Allocate",
                                              offsets::kFunc_BethesdaAlloc,
                                              std::array<std::uint8_t, 10>{ 0x48, 0x89, 0x5C, 0x24, 0x10, 0x48, 0x89, 0x6C, 0x24, 0x18 }) &&
                    validateNativeEntry(
                        "MemoryManager initialization",
                        offsets::kFunc_BethesdaAllocatorInit,
                        std::array<std::uint8_t, 12>{ 0x57, 0x48, 0x83, 0xEC, 0x20, 0x44, 0x8B, 0x05, 0xEF, 0xAC, 0xD0, 0x04 }) &&
                    validateNativeEntry(
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
