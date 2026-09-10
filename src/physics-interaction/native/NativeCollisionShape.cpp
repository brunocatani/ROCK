#include "physics-interaction/native/NativeCollisionShape.h"

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"

#include <array>
#include <limits>
#include <windows.h>

namespace rock::native_scene
{
    namespace
    {
        using CollisionObjectQuery = RE::bhkNPCollisionObject* (*)(RE::NiCollisionObject*);
        using CollisionShapeGetter = RE::hknpShape* (*)(RE::bhkNPCollisionObject*);

        struct NativeEntries
        {
            CollisionObjectQuery query{ nullptr };
            CollisionShapeGetter shape{ nullptr };
        };

        template <std::size_t N>
        bool entryMatches(const std::uintptr_t address, const std::array<std::uint8_t, N>& expected)
        {
            const auto text = REL::Module::get().segment(REL::Segment::text);
            std::array<std::uint8_t, N> actual{};
            return address >= text.address() && address + N <= text.address() + text.size() &&
                native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(address), actual.data(), actual.size()) &&
                actual == expected;
        }

        const NativeEntries& nativeEntries()
        {
            static const NativeEntries entries = []() -> NativeEntries {
                if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) {
                    ROCK_LOG_ERROR(Weapon, "Collision shape query rejected: unsupported runtime");
                    return {};
                }
                const auto query = REL::Offset(0x1E07980).address();
                const auto shape = REL::Offset(offsets::kFunc_CollisionObject_GetShape).address();
                if (!entryMatches(query, std::array<std::uint8_t, 4>{ 0x48, 0x8B, 0xC1, 0xC3 }) ||
                    !entryMatches(shape, std::array<std::uint8_t, 13>{
                        0x40, 0x53, 0x48, 0x83, 0xEC, 0x20, 0x48, 0x8B, 0xD9, 0x48, 0x8B, 0x49, 0x20 })) {
                    ROCK_LOG_ERROR(Weapon, "Collision shape query rejected: native entry bytes differ");
                    return {};
                }
                return { reinterpret_cast<CollisionObjectQuery>(query), reinterpret_cast<CollisionShapeGetter>(shape) };
            }();
            return entries;
        }

        RE::hknpShape* guardedGetShape(const CollisionShapeGetter getter, RE::bhkNPCollisionObject* collision)
        {
            // A scene owner can survive native body retirement. The engine's
            // getter dereferences its resolved body without a null check.
            __try {
                return getter(collision);
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return nullptr;
            }
        }
    }

    CollisionShapeQueryResult queryCollisionShape(RE::NiAVObject* node)
    {
        RE::NiCollisionObject* collision = nullptr;
        if (!native_memory::tryReadField(node, offsets::kNiAVObject_CollisionObject, collision)) {
            return {};
        }
        if (!collision) {
            return { nullptr, CollisionShapeQueryStage::CollisionObject };
        }

        // FO4VR constructors 141E07710/141E0B0E0 install the table whose
        // +D8 entry is 141E07980 (return this). Native traversal 141E1B1A0
        // calls +D8. CommonLib's +C8 calls a null-returning method instead.
        // Check the entry identity, not one concrete vtable: native derived
        // collision wrappers inherit the same query.
        const auto& entries = nativeEntries();
        std::uintptr_t vtable = 0;
        CollisionObjectQuery query = nullptr;
        if (!entries.query ||
            !native_memory::tryReadField(collision, 0, vtable) ||
            !native_memory::tryReadField(reinterpret_cast<const void*>(vtable), 0xD8, query) ||
            query != entries.query) {
            return { nullptr, CollisionShapeQueryStage::NativeDispatch };
        }
        auto* nativeCollision = query(collision);
        void* system = nullptr;
        std::uint32_t bodyIndex = std::numeric_limits<std::uint32_t>::max();
        if (nativeCollision != reinterpret_cast<RE::bhkNPCollisionObject*>(collision) ||
            !native_memory::tryReadField(nativeCollision, offsets::kCollisionObject_PhysSystemPtr, system) ||
            !native_memory::pointerRangeLooksReadable(system, 0x20) ||
            !native_memory::tryReadField(nativeCollision, offsets::kCollisionObject_SystemBodyIndex, bodyIndex) ||
            bodyIndex > static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max())) {
            return { nullptr, CollisionShapeQueryStage::PhysicsSystem };
        }

        // Keep the native live-body/template selection; do not create a body
        // in the world merely to inspect serialized collision materials.
        auto* shape = guardedGetShape(entries.shape, nativeCollision);
        if (!native_memory::pointerRangeLooksReadable(shape, sizeof(RE::hknpShape))) {
            return { nullptr, CollisionShapeQueryStage::Shape };
        }
        return { shape, CollisionShapeQueryStage::Complete };
    }
}
