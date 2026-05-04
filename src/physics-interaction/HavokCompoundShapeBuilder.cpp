#include "HavokCompoundShapeBuilder.h"

#include "HavokOffsets.h"
#include "HavokRuntime.h"
#include "PhysicsLog.h"

#include <REL/Relocation.h>

#include <intrin.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <vector>

namespace frik::rock::havok_compound_shape_builder
{
    namespace
    {
        constexpr std::size_t kShapeInstanceSize = 0x80;
        constexpr std::size_t kCompoundShapeCinfoSize = 0x28;
        constexpr std::size_t kStaticCompoundShapeSize = 0xD0;

        struct alignas(16) ShapeInstance
        {
            std::array<std::byte, kShapeInstanceSize> bytes{};
        };

        struct CompoundShapeCinfo
        {
            ShapeInstance* instances = nullptr;
            std::int32_t count = 0;
            std::int32_t capacityAndFlags = 0;
            std::uint8_t flags = 0;
            std::array<std::byte, 7> pad11{};
            void* massConfig = nullptr;
            void* outputIds = nullptr;
        };

        static_assert(sizeof(Vector4) == 0x10);
        static_assert(sizeof(ChildTransform) == 0x60);
        static_assert(offsetof(ChildTransform, scale) == 0x40);
        static_assert(sizeof(ShapeInstance) == kShapeInstanceSize);
        static_assert(alignof(ShapeInstance) == 0x10);
        static_assert(sizeof(CompoundShapeCinfo) == kCompoundShapeCinfoSize);
        static_assert(offsetof(CompoundShapeCinfo, massConfig) == 0x18);
        static_assert(offsetof(CompoundShapeCinfo, outputIds) == 0x20);

        using CompoundCinfoCtor_t = CompoundShapeCinfo* (*)(CompoundShapeCinfo*, ShapeInstance*, std::int32_t, void*);
        using StaticCompoundCtor_t = RE::hknpShape* (*)(void*, CompoundShapeCinfo*, std::uint64_t, void*);
        using SetShape_t = void (*)(ShapeInstance*, const RE::hknpShape*);
        using SetTransform_t = void (*)(ShapeInstance*, const ChildTransform*);
        using SetScale_t = void (*)(ShapeInstance*, const Vector4*, int);

        void releaseShapeReference(const RE::hknpShape* shape) noexcept
        {
            if (!shape) {
                return;
            }

            auto* refCountDword = reinterpret_cast<volatile long*>(const_cast<char*>(reinterpret_cast<const char*>(shape)) + 0x08);
            for (;;) {
                const long oldValue = *refCountDword;
                const auto oldRef = static_cast<std::uint16_t>(oldValue & 0xFFFF);
                if (oldRef == 0xFFFF || oldRef == 0) {
                    return;
                }

                const auto newRef = static_cast<std::uint16_t>(oldRef - 1);
                const long newValue = (oldValue & static_cast<long>(0xFFFF0000u)) | static_cast<long>(newRef);
                if (_InterlockedCompareExchange(refCountDword, newValue, oldValue) == oldValue) {
                    return;
                }
            }
        }

        void initializeDefaultInstance(ShapeInstance& instance) noexcept
        {
            instance.bytes.fill(std::byte{ 0 });
            *reinterpret_cast<std::uint32_t*>(instance.bytes.data() + 0x58) = 0xFFFFFFFFu;
            *reinterpret_cast<std::uint32_t*>(instance.bytes.data() + 0x0C) = 0x3F000040u;
        }

        const RE::hknpShape* instanceShape(const ShapeInstance& instance) noexcept
        {
            return *reinterpret_cast<const RE::hknpShape* const*>(instance.bytes.data() + 0x50);
        }

        void releaseTemporaryInstanceShapeReferences(std::span<const ShapeInstance> instances) noexcept
        {
            for (const auto& instance : instances) {
                releaseShapeReference(instanceShape(instance));
            }
        }

        bool validateChildren(std::span<const CompoundChild> children) noexcept
        {
            if (children.empty()) {
                ROCK_LOG_WARN(Weapon, "Static compound build skipped: no child shapes");
                return false;
            }

            if (children.size() > kMaxStaticCompoundChildren ||
                children.size() > static_cast<std::size_t>((std::numeric_limits<std::int32_t>::max)())) {
                ROCK_LOG_WARN(Weapon, "Static compound build skipped: child count {} exceeds supported maximum {}", children.size(), kMaxStaticCompoundChildren);
                return false;
            }

            for (std::size_t i = 0; i < children.size(); ++i) {
                if (!children[i].shape) {
                    ROCK_LOG_WARN(Weapon, "Static compound build skipped: child {} has a null shape", i);
                    return false;
                }
            }

            return true;
        }
    }

    RE::hknpShape* buildStaticCompoundShape(std::span<const CompoundChild> children) noexcept
    {
        if (!validateChildren(children)) {
            return nullptr;
        }

        std::vector<ShapeInstance> instances(children.size());

        static REL::Relocation<SetTransform_t> setTransform{ REL::Offset(offsets::kFunc_ShapeInstance_SetTransform) };
        static REL::Relocation<SetScale_t> setScale{ REL::Offset(offsets::kFunc_ShapeInstance_SetScale) };
        static REL::Relocation<SetShape_t> setShape{ REL::Offset(offsets::kFunc_ShapeInstance_SetShape) };

        for (std::size_t i = 0; i < children.size(); ++i) {
            auto& instance = instances[i];
            const auto& child = children[i];

            initializeDefaultInstance(instance);
            setTransform(std::addressof(instance), std::addressof(child.transform));
            setScale(std::addressof(instance), std::addressof(child.transform.scale), child.transform.scaleMode);
            setShape(std::addressof(instance), child.shape);
        }

        CompoundShapeCinfo cinfo;
        const auto childCount = static_cast<std::int32_t>(instances.size());
        static REL::Relocation<CompoundCinfoCtor_t> constructCinfo{ REL::Offset(offsets::kFunc_CompoundShapeCinfo_FromInstances) };
        constructCinfo(std::addressof(cinfo), instances.data(), childCount, nullptr);

        auto* storage = havok_runtime::allocateHavok(kStaticCompoundShapeSize);
        if (!storage) {
            releaseTemporaryInstanceShapeReferences(instances);
            ROCK_LOG_WARN(Weapon, "Static compound build failed: Havok heap allocation returned null");
            return nullptr;
        }

        std::memset(storage, 0, kStaticCompoundShapeSize);

        static REL::Relocation<StaticCompoundCtor_t> constructStaticCompound{ REL::Offset(offsets::kFunc_StaticCompoundShape_Ctor) };
        auto* compound = constructStaticCompound(storage, std::addressof(cinfo), static_cast<std::uint64_t>(childCount), nullptr);
        releaseTemporaryInstanceShapeReferences(instances);

        if (!compound) {
            havok_runtime::freeHavok(storage, kStaticCompoundShapeSize);
            ROCK_LOG_WARN(Weapon, "Static compound build failed: native constructor returned null for {} children", children.size());
            return nullptr;
        }

        return compound;
    }
}
