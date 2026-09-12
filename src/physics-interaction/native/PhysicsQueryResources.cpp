#include "physics-interaction/native/PhysicsQueryResources.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/HavokRefCount.h"
#include "physics-interaction/native/NativeMemory.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace rock::physics_query_resources
{
    namespace
    {
        using DestroyCollector = void* (*)(RE::hknpAllHitsCollector*, std::uint32_t);

        // FO4VR 1.2.72: PickObject (141DF8E76/141DF8ED6) retains the world at
        // pick+0xC0; native caller 1407F89BD..1407F8A03 releases it at scope exit.
        // CommonLib misnames this member and its comments omit alignment padding.
        static_assert(offsetof(RE::bhkPickData, allHitsResult) == 0xC0);

        template <std::size_t N>
        bool matches(std::uintptr_t rva, const std::array<std::uint8_t, N>& expected)
        {
            std::array<std::uint8_t, N> actual{};
            return native_memory::guardedCopyFromMemory(
                       reinterpret_cast<const void*>(REL::Offset(rva).address()),
                       actual.data(), actual.size()) && actual == expected;
        }

        DestroyCollector collectorDestructor() noexcept
        {
            static const auto destroy = []() -> DestroyCollector {
                if (!REL::Module::IsVR() ||
                    REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) {
                    ROCK_LOG_ERROR(PhysicsSafety, "Native query cleanup rejected: unsupported runtime");
                    return nullptr;
                }
                // Collector destructor 140066E80 and inlined cleanup 1410563A7
                // agree: free positive-capacity storage through Havok's allocator;
                // the deleting-destructor flag must be zero for our stack object.
                const auto destructor = REL::Offset(0x066E80).address();
                std::uintptr_t vtableEntry = 0;
                if (!matches(0x1DF8E76, std::array<std::uint8_t, 16>{
                        0x66, 0x39, 0x5E, 0x0A, 0x74, 0x18, 0x0F, 0x0D,
                        0x4E, 0x08, 0x8B, 0x46, 0x08, 0x8D, 0x48, 0x01 }) ||
                    !matches(0x1DF8ED6, std::array<std::uint8_t, 7>{
                        0x49, 0x89, 0xB6, 0xC0, 0x00, 0x00, 0x00 }) ||
                    !matches(0x066E80, std::array<std::uint8_t, 10>{
                        0x48, 0x89, 0x5C, 0x24, 0x08, 0x57, 0x48, 0x83, 0xEC, 0x20 }) ||
                    !native_memory::tryReadValue(
                        reinterpret_cast<const std::uintptr_t*>(REL::Offset(0x2C838C8).address()),
                        vtableEntry) || vtableEntry != destructor) {
                    ROCK_LOG_ERROR(PhysicsSafety, "Native query cleanup rejected: ownership validation failed");
                    return nullptr;
                }
                ROCK_LOG_INFO(PhysicsSafety, "Native query cleanup ready: scoped world references and collector overflow storage");
                return reinterpret_cast<DestroyCollector>(destructor);
            }();
            return destroy;
        }
    }

    bool nativeCleanupReady() noexcept
    {
        return collectorDestructor() != nullptr;
    }

    PickData::~PickData() noexcept
    {
        auto* world = std::exchange(allHitsResult, nullptr);
        if (!world) {
            return;
        }

        // Per-thread diagnostic counters retain no engine pointers. Reference
        // words are snapshots: other native queries may acquire/release in parallel.
        static thread_local std::uint64_t cleanupCount = 0;
        const auto sequence = ++cleanupCount;
        const bool trace = logger::isDebugEnabled() && (sequence <= 4 || sequence % 4096 == 0);
        std::uint32_t referenceWord = 0;
        const bool readable = trace && native_memory::tryReadField(world, 0x08, referenceWord);
        havok_ref_count::release(world);
        if (trace) {
            ROCK_LOG_DEBUG(PhysicsSafety,
                "Raycast world cleanup: count={} world={:p} refBefore=0x{:08X} readable={} pickCleared=true",
                sequence, static_cast<void*>(world), referenceWord, readable);
        }
    }

    AllHitsCollector::~AllHitsCollector() noexcept
    {
        if (_native.hits._capacityAndFlags < 0) {
            return;  // Inline storage is part of this stack object.
        }
        const auto destroy = collectorDestructor();
        if (!destroy) {
            return;  // Queries cannot run when nativeCleanupReady() is false.
        }
        const auto capacity = static_cast<std::uint32_t>(_native.hits._capacityAndFlags) & 0x3FFF'FFFFu;
        destroy(&_native, 0);
        ROCK_LOG_SAMPLE_DEBUG(PhysicsSafety, 5000,
            "Query collector overflow cleanup: capacity={} bytes={}",
            capacity, static_cast<std::uint64_t>(capacity) * sizeof(RE::hknpCollisionResult));
    }
}
