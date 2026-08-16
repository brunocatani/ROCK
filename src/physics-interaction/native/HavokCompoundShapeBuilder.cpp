#include "physics-interaction/native/HavokCompoundShapeBuilder.h"

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokRefCount.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"

#include <REL/Relocation.h>

#include <intrin.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <vector>

namespace rock::havok_compound_shape_builder
{
    namespace
    {
        constexpr std::size_t kShapeInstanceSize = 0x80;
        constexpr std::size_t kCompoundShapeCinfoSize = 0x28;
        constexpr std::size_t kStaticCompoundShapeSize = 0xD0;
        constexpr std::size_t kDynamicCompoundShapeSize = 0xD0;
        constexpr float kTransformComparisonEpsilon = 0.00001f;

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
        using DynamicCompoundCtor_t = RE::hknpShape* (*)(void*, CompoundShapeCinfo*);
        using SetShape_t = void (*)(ShapeInstance*, const RE::hknpShape*);
        using SetTransform_t = void (*)(ShapeInstance*, const ChildTransform*);
        using SetScale_t = void (*)(ShapeInstance*, const Vector4*, int);
        using UpdateInstances_t = void (*)(RE::hknpShape*, const std::int16_t*, std::int32_t, const ShapeInstance*);

        struct NativeApi
        {
            CompoundCinfoCtor_t constructCinfo = nullptr;
            DynamicCompoundCtor_t constructDynamicCompound = nullptr;
            SetShape_t setShape = nullptr;
            SetTransform_t setTransform = nullptr;
            SetScale_t setScale = nullptr;
            UpdateInstances_t updateInstances = nullptr;

            [[nodiscard]] bool dynamicReady() const noexcept
            {
                return constructCinfo && constructDynamicCompound && setShape && setTransform && setScale && updateInstances;
            }
        };

        [[nodiscard]] bool addressIsInGameText(const std::uintptr_t address) noexcept
        {
            const auto text = REL::Module::get().segment(REL::Segment::text);
            return address >= text.address() && address < text.address() + text.size();
        }

        template <std::size_t N>
        [[nodiscard]] bool validateNativeEntry(
            const char* label,
            const std::uintptr_t offset,
            const std::array<std::uint8_t, N>& expected,
            const std::array<std::uint8_t, N>& mask = [] {
                std::array<std::uint8_t, N> result{};
                result.fill(0xFF);
                return result;
            }()) noexcept
        {
            const auto address = REL::Offset(offset).address();
            std::array<std::uint8_t, N> actual{};
            if (!addressIsInGameText(address) ||
                !native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(address), actual.data(), actual.size())) {
                ROCK_LOG_ERROR(Init, "Native compound-shape validation could not read {} at 0x{:X}", label, address);
                return false;
            }
            for (std::size_t i = 0; i < N; ++i) {
                if ((actual[i] & mask[i]) != (expected[i] & mask[i])) {
                    ROCK_LOG_ERROR(Init, "Native compound-shape validation failed for {} at 0x{:X}", label, address);
                    return false;
                }
            }
            return true;
        }

        [[nodiscard]] const NativeApi& nativeApi() noexcept
        {
            static const NativeApi api = []() noexcept {
                NativeApi result{};
                if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) {
                    ROCK_LOG_ERROR(Init, "Native compound shapes require the verified Fallout4VR.exe 1.2.72 layout");
                    return result;
                }

                /*
                 * Blind-verified from raw Fallout4VR.exe 1.2.72 disassembly on
                 * 2026-08-09. The dynamic update entry copies each full 0x80
                 * instance, refreshes its dynamic-tree leaf and aggregate AABB,
                 * then signals registered shape owners about the mutation.
                 */
                const bool entriesMatch =
                    validateNativeEntry(
                        "hknpShapeInstance::setShape",
                        offsets::kFunc_ShapeInstance_SetShape,
                        std::array<std::uint8_t, 10>{ 0x48, 0x89, 0x5C, 0x24, 0x08, 0x57, 0x48, 0x83, 0xEC, 0x20 }) &&
                    validateNativeEntry(
                        "hknpShapeInstance::setTransform",
                        offsets::kFunc_ShapeInstance_SetTransform,
                        std::array<std::uint8_t, 15>{ 0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x6C, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18 }) &&
                    validateNativeEntry(
                        "hknpShapeInstance::setScale",
                        offsets::kFunc_ShapeInstance_SetScale,
                        std::array<std::uint8_t, 14>{ 0x0F, 0x28, 0x12, 0x0F, 0x28, 0x1D, 0x00, 0x00, 0x00, 0x00, 0x44, 0x8B, 0x49, 0x0C },
                        std::array<std::uint8_t, 14>{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF }) &&
                    validateNativeEntry(
                        "hknpCompoundShapeCinfo constructor",
                        offsets::kFunc_CompoundShapeCinfo_FromInstances,
                        std::array<std::uint8_t, 12>{ 0x33, 0xC0, 0x48, 0x89, 0x11, 0x44, 0x89, 0x41, 0x08, 0x89, 0x41, 0x0C }) &&
                    validateNativeEntry(
                        "hknpDynamicCompoundShape constructor",
                        offsets::kFunc_DynamicCompoundShape_Ctor,
                        std::array<std::uint8_t, 15>{ 0x48, 0x89, 0x5C, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x57, 0x48, 0x83, 0xEC, 0x60 }) &&
                    validateNativeEntry(
                        "hknpDynamicCompoundShape::updateInstances",
                        offsets::kFunc_DynamicCompoundShape_UpdateInstances,
                        std::array<std::uint8_t, 17>{ 0x48, 0x89, 0x5C, 0x24, 0x18, 0x55, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x83, 0xEC });
                if (!entriesMatch) {
                    return result;
                }

                result.constructCinfo = reinterpret_cast<CompoundCinfoCtor_t>(REL::Offset(offsets::kFunc_CompoundShapeCinfo_FromInstances).address());
                result.constructDynamicCompound = reinterpret_cast<DynamicCompoundCtor_t>(REL::Offset(offsets::kFunc_DynamicCompoundShape_Ctor).address());
                result.setShape = reinterpret_cast<SetShape_t>(REL::Offset(offsets::kFunc_ShapeInstance_SetShape).address());
                result.setTransform = reinterpret_cast<SetTransform_t>(REL::Offset(offsets::kFunc_ShapeInstance_SetTransform).address());
                result.setScale = reinterpret_cast<SetScale_t>(REL::Offset(offsets::kFunc_ShapeInstance_SetScale).address());
                result.updateInstances = reinterpret_cast<UpdateInstances_t>(REL::Offset(offsets::kFunc_DynamicCompoundShape_UpdateInstances).address());
                return result;
            }();
            return api;
        }

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
                ROCK_LOG_WARN(Weapon, "Compound build skipped: no child shapes");
                return false;
            }

            if (children.size() > kMaxStaticCompoundChildren ||
                children.size() > static_cast<std::size_t>((std::numeric_limits<std::int32_t>::max)())) {
                ROCK_LOG_WARN(Weapon, "Compound build skipped: child count {} exceeds supported maximum {}", children.size(), kMaxStaticCompoundChildren);
                return false;
            }

            for (std::size_t i = 0; i < children.size(); ++i) {
                if (!children[i].shape) {
                    ROCK_LOG_WARN(Weapon, "Compound build skipped: child {} has a null shape", i);
                    return false;
                }
            }

            return true;
        }

        [[nodiscard]] bool vectorNearlyEqual(const Vector4& lhs, const Vector4& rhs) noexcept
        {
            return std::abs(lhs.x - rhs.x) <= kTransformComparisonEpsilon &&
                   std::abs(lhs.y - rhs.y) <= kTransformComparisonEpsilon &&
                   std::abs(lhs.z - rhs.z) <= kTransformComparisonEpsilon &&
                   std::abs(lhs.w - rhs.w) <= kTransformComparisonEpsilon;
        }

        [[nodiscard]] bool transformsNearlyEqual(const ChildTransform& lhs, const ChildTransform& rhs) noexcept
        {
            return vectorNearlyEqual(lhs.column0, rhs.column0) &&
                   vectorNearlyEqual(lhs.column1, rhs.column1) &&
                   vectorNearlyEqual(lhs.column2, rhs.column2) &&
                   vectorNearlyEqual(lhs.translation, rhs.translation) &&
                   vectorNearlyEqual(lhs.scale, rhs.scale) &&
                   lhs.scaleMode == rhs.scaleMode;
        }

        [[nodiscard]] bool transformIsFinite(const ChildTransform& transform) noexcept
        {
            const auto finiteVector = [](const Vector4& value) noexcept {
                return std::isfinite(value.x) && std::isfinite(value.y) &&
                       std::isfinite(value.z) && std::isfinite(value.w);
            };
            return finiteVector(transform.column0) && finiteVector(transform.column1) &&
                   finiteVector(transform.column2) && finiteVector(transform.translation) &&
                   finiteVector(transform.scale);
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

    void HavokShapeRelease::operator()(RE::hknpShape* shape) const noexcept
    {
        havok_ref_count::release(shape);
    }

    bool DynamicCompoundShape::create(std::span<const CompoundChild> children) noexcept
    {
        reset();
        if (!validateChildren(children)) {
            return false;
        }
        for (std::size_t i = 0; i < children.size(); ++i) {
            if (!transformIsFinite(children[i].transform)) {
                ROCK_LOG_WARN(Weapon, "Dynamic compound build skipped: child {} has a nonfinite transform", i);
                return false;
            }
        }

        const auto& api = nativeApi();
        if (!api.dynamicReady()) {
            ROCK_LOG_WARN(Weapon, "Dynamic compound build failed: native API validation unavailable");
            return false;
        }

        _instances.resize(children.size());
        _instanceIds.assign(children.size(), static_cast<std::int16_t>(0x7FFF));
        _lastTransforms.resize(children.size());
        _updateInstances.reserve(children.size());
        _updateIds.reserve(children.size());

        auto* instances = reinterpret_cast<ShapeInstance*>(_instances.data());
        for (std::size_t i = 0; i < children.size(); ++i) {
            initializeDefaultInstance(instances[i]);
            api.setTransform(std::addressof(instances[i]), std::addressof(children[i].transform));
            api.setScale(std::addressof(instances[i]), std::addressof(children[i].transform.scale), children[i].transform.scaleMode);
            api.setShape(std::addressof(instances[i]), children[i].shape);
            _lastTransforms[i] = children[i].transform;
        }

        CompoundShapeCinfo cinfo;
        const auto childCount = static_cast<std::int32_t>(children.size());
        api.constructCinfo(std::addressof(cinfo), instances, childCount, nullptr);
        cinfo.outputIds = _instanceIds.data();

        auto* storage = havok_runtime::allocateHavok(kDynamicCompoundShapeSize);
        if (!storage) {
            releaseTemporaryInstanceShapeReferences(std::span<const ShapeInstance>{ instances, children.size() });
            reset();
            ROCK_LOG_WARN(Weapon, "Dynamic compound build failed: Havok heap allocation returned null");
            return false;
        }
        std::memset(storage, 0, kDynamicCompoundShapeSize);

        auto* compound = api.constructDynamicCompound(storage, std::addressof(cinfo));
        releaseTemporaryInstanceShapeReferences(std::span<const ShapeInstance>{ instances, children.size() });
        if (!compound) {
            havok_runtime::freeHavok(storage, kDynamicCompoundShapeSize);
            reset();
            ROCK_LOG_WARN(Weapon, "Dynamic compound build failed: native constructor returned null for {} children", children.size());
            return false;
        }

        _shape.reset(compound);
        _shapeKeyBitCount = compoundShapeKeyBitCount(children.size());
        const auto maximumEncodedInstanceId =
            (1u << _shapeKeyBitCount) - 1u;
        for (std::size_t i = 0; i < _instanceIds.size(); ++i) {
            if (_instanceIds[i] < 0 || _instanceIds[i] >= 0x7FFF ||
                static_cast<std::uint32_t>(_instanceIds[i]) >
                    maximumEncodedInstanceId) {
                ROCK_LOG_ERROR(
                    Weapon,
                    "Dynamic compound build failed: constructor returned unencodable child id {} at index {} for {} key bits",
                    _instanceIds[i],
                    i,
                    _shapeKeyBitCount);
                reset();
                return false;
            }
            for (std::size_t previous = 0; previous < i; ++previous) {
                if (_instanceIds[previous] == _instanceIds[i]) {
                    ROCK_LOG_ERROR(Weapon, "Dynamic compound build failed: constructor returned duplicate child id {}", _instanceIds[i]);
                    reset();
                    return false;
                }
            }
        }
        return true;
    }

    std::optional<std::size_t> DynamicCompoundShape::tryResolveChildIndex(
        const std::uint32_t shapeKey) const noexcept
    {
        if (!_shape || shapeKey == 0xFFFF'FFFFu ||
            _shapeKeyBitCount == 0 || _shapeKeyBitCount >= 32 ||
            _instanceIds.size() != _instances.size()) {
            return std::nullopt;
        }

        const auto decodedId = decodeTopLevelCompoundInstanceId(
            shapeKey,
            _shapeKeyBitCount);
        if (!decodedId) {
            return std::nullopt;
        }
        const auto nativeInstanceId = static_cast<std::int16_t>(*decodedId);
        const auto found = std::find(
            _instanceIds.begin(),
            _instanceIds.end(),
            nativeInstanceId);
        if (found == _instanceIds.end()) {
            return std::nullopt;
        }
        return static_cast<std::size_t>(
            std::distance(_instanceIds.begin(), found));
    }

    DynamicCompoundUpdateResult DynamicCompoundShape::updateTransforms(std::span<const ChildTransform> transforms) noexcept
    {
        DynamicCompoundUpdateResult result{};
        if (!_shape || transforms.size() != _instances.size() || _instanceIds.size() != _instances.size()) {
            return result;
        }
        if (std::any_of(transforms.begin(), transforms.end(), [](const ChildTransform& transform) {
                return !transformIsFinite(transform);
            })) {
            return result;
        }

        const auto& api = nativeApi();
        if (!api.dynamicReady()) {
            return result;
        }

        _updateInstances.clear();
        _updateIds.clear();
        auto* instances = reinterpret_cast<ShapeInstance*>(_instances.data());
        for (std::size_t i = 0; i < transforms.size(); ++i) {
            if (transformsNearlyEqual(_lastTransforms[i], transforms[i])) {
                continue;
            }

            api.setTransform(std::addressof(instances[i]), std::addressof(transforms[i]));
            api.setScale(std::addressof(instances[i]), std::addressof(transforms[i].scale), transforms[i].scaleMode);
            _lastTransforms[i] = transforms[i];
            _updateInstances.push_back(_instances[i]);
            _updateIds.push_back(_instanceIds[i]);
        }

        result.changedChildCount = _updateIds.size();
        if (!_updateIds.empty()) {
            api.updateInstances(
                _shape.get(),
                _updateIds.data(),
                static_cast<std::int32_t>(_updateIds.size()),
                reinterpret_cast<const ShapeInstance*>(_updateInstances.data()));
        }
        result.succeeded = true;
        return result;
    }

    void DynamicCompoundShape::reset() noexcept
    {
        _shapeKeyBitCount = 0;
        _updateIds.clear();
        _updateInstances.clear();
        _lastTransforms.clear();
        _instanceIds.clear();
        _instances.clear();
        _shape.reset();
    }
}
