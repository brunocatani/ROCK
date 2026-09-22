#include "physics-interaction/grenade/LooseMolotovVisual.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/grab/HeldScenePresentationPolicy.h"
#include "physics-interaction/grenade/LooseMolotovVisualPolicy.h"
#include "physics-interaction/native/NativeMemory.h"
#include "rock_support/Fo4VrRuntime.h"

#include "RE/Bethesda/BSGeometry.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/NetImmerse/NiCloningProcess.h"
#include "RE/NetImmerse/NiProperty.h"
#include "RE/NetImmerse/NiUpdateData.h"

#include <cstring>
#include <exception>

namespace rock
{
    namespace
    {
        constexpr std::array kShapeNames{
            "baseGlow001:0", "WickFX:0", "flameSmall:0", "flameSmall:1", "FlameBig:0"
        };
        constexpr std::size_t kMaxControllersPerProperty = 4;
        constexpr std::uintptr_t kEffectPropertyVtable = 0x30A5938;
        constexpr std::uintptr_t kFloatControllerVtable = 0x30BFE88;

        using LoadModel = int (*)(const char*, RE::NiPointer<RE::NiNode>*, const std::uint64_t*);
        using Clone = RE::NiObject* (*)(RE::NiObject*, RE::NiCloningProcess*);
        using StartController = void (*)(RE::NiTimeController*, float);
        using UpdateController = void (*)(RE::NiTimeController*, RE::NiUpdateData*);

        struct Resources
        {
            RE::NiPointer<RE::NiNode> model;
            RE::NiTransform wickLocal{};
            RE::NiBound wickBound{};
            std::array<RE::NiPointer<RE::BSTriShape>, kShapeNames.size()> shapes;
        };

        // Explicitly replaced on GameLoaded/NewGame/PostLoadGame, after the old
        // PhysicsInteraction has released its armed slots. No initialization I/O.
        std::unique_ptr<Resources> resources;
        Clone cloneObject = nullptr;
        StartController startController = nullptr;
        UpdateController updateController = nullptr;

        template <std::size_t N>
        bool verifiedEntry(std::uintptr_t rva, const std::array<std::uint8_t, N>& expected)
        {
            std::array<std::uint8_t, N> actual{};
            return native_memory::guardedCopyFromMemory(
                reinterpret_cast<const void*>(REL::Module::get().base() + rva), actual.data(), actual.size()) &&
                actual == expected;
        }

        bool verifyNativeCalls()
        {
            // FO4VR 1.2.72, raw-disassembly witnesses:
            // 1402D9140 and 140723960 construct the 0x78 clone process and call
            // 141C13FF0 (CreateClone + ProcessClone). 141C31050 clones geometry
            // properties; 14278E0C0 gives animated properties private materials.
            // 141C25AD0 remaps controller targets to the cloned property.
            // 141C2A620 dispatches the float controller's update slot, installed
            // by 14292CC70; 14292CF80 consumes NiUpdateData.time. 141C26060 and
            // 141C26400 agree on start/last-time fields. Never trust flat slots.
            const bool valid =
                verifiedEntry(0x1D0DEE0, std::to_array<std::uint8_t>({
                    0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x30,0x48,0x8B,0xDA,0x48,0x8D,0x54,0x24,0x20,
                    0x48,0xC7,0x44,0x24,0x20,0,0,0,0 })) &&
                verifiedEntry(0x1C13FF0, std::to_array<std::uint8_t>({
                    0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x74,0x24,0x10,0x57,0x48,0x83,0xEC,0x20,
                    0x48,0x8B,0x01,0x48,0x8B,0xFA,0x48,0x8B,0xF1,0xFF,0x90,0xE0,0,0,0 })) &&
                verifiedEntry(0x1C26060, std::to_array<std::uint8_t>({
                    0x66,0x83,0x49,0x10,0x08,0xF6,0x41,0x10,0x01,0xC7,0x41,0x28,0xFF,0xFF,0x7F,0xFF })) &&
                verifiedEntry(0x292CF80, std::to_array<std::uint8_t>({
                    0x40,0x53,0x48,0x83,0xEC,0x20,0x0F,0xB6,0x41,0x10,0x48,0x8B,0xD9,0xC0,0xE8,0x05,0xA8,0x01 }));
            if (valid) {
                const auto base = REL::Module::get().base();
                cloneObject = reinterpret_cast<Clone>(base + 0x1C13FF0);
                startController = reinterpret_cast<StartController>(base + 0x1C26060);
                updateController = reinterpret_cast<UpdateController>(base + 0x292CF80);
            }
            return valid;
        }

        bool hasVtable(const void* object, std::uintptr_t rva)
        {
            std::uintptr_t vtable = 0;
            return native_memory::tryReadValue(static_cast<const std::uintptr_t*>(object), vtable) &&
                vtable == REL::Module::get().base() + rva;
        }

        // Fixed traversal budget, including sparse child slots. Called on scene
        // acquisition only; unsupported/rejected roots are not searched per frame.
        RE::BSTriShape* findShape(RE::NiAVObject* root, const char* name)
        {
            std::array<RE::NiAVObject*, 128> pending{};
            std::size_t count = 1;
            std::size_t visitedSlots = 0;
            pending[0] = root;
            RE::BSTriShape* result = nullptr;
            for (std::size_t index = 0; index < count; ++index) {
                auto* object = pending[index];
                if (!object) continue;
                if (object->name.c_str() && std::strcmp(object->name.c_str(), name) == 0) {
                    if (result) return nullptr;
                    result = object->IsTriShape();
                    if (!result) return nullptr;
                }
                if (auto* node = object->IsNode()) {
                    for (const auto& child : node->children) {
                        if (++visitedSlots > pending.size()) return nullptr;
                        if (!child) continue;
                        if (count == pending.size()) return nullptr;
                        pending[count++] = child.get();
                    }
                }
            }
            return result;
        }

        bool validControllerChain(RE::NiTimeController* controller, RE::NiProperty* property)
        {
            std::size_t count = 0;
            for (; controller; controller = controller->next.get()) {
                if (++count > kMaxControllersPerProperty || !hasVtable(controller, kFloatControllerVtable) ||
                    !native_memory::pointerRangeLooksReadable(controller, 0x58) || controller->target != property ||
                    !std::isfinite(controller->frequency) || !std::isfinite(controller->phase) ||
                    !std::isfinite(controller->loKeyTime) || !std::isfinite(controller->hiKeyTime)) return false;
                void* interpolator = nullptr;
                std::uint32_t variable = 0;
                if (!native_memory::tryReadField(controller, 0x48, interpolator) ||
                    !native_memory::pointerLooksReadable(interpolator) ||
                    !native_memory::tryReadField(controller, 0x50, variable) || variable > 8) return false;
            }
            return true;
        }

        bool qualifyProperties(RE::BSTriShape* shape, const RE::BSTriShape* source = nullptr)
        {
            bool animated = false;
            for (std::size_t index = 0; index < 2; ++index) {
                auto* property = shape->properties[index].get();
                if (!property || !property->controllers) continue;
                if (!hasVtable(property, kEffectPropertyVtable) ||
                    !validControllerChain(property->controllers.get(), property)) return false;
                void* material = nullptr;
                if (!native_memory::tryReadField(property, 0x58, material) ||
                    !native_memory::pointerLooksReadable(material)) return false;
                if (source) {
                    auto* original = source->properties[index].get();
                    void* originalMaterial = nullptr;
                    if (!original || original == property || original->controllers == property->controllers ||
                        !native_memory::tryReadField(original, 0x58, originalMaterial) || material == originalMaterial) return false;
                    auto* originalController = original->controllers.get();
                    for (auto* controller = property->controllers.get(); controller; controller = controller->next.get()) {
                        void* interpolator = nullptr;
                        void* originalInterpolator = nullptr;
                        if (!originalController || controller == originalController ||
                            !native_memory::tryReadField(controller, 0x48, interpolator) ||
                            !native_memory::tryReadField(originalController, 0x48, originalInterpolator) ||
                            interpolator == originalInterpolator) return false;
                        originalController = originalController->next.get();
                    }
                    if (originalController) return false;
                }
                animated = true;
            }
            return animated;
        }
    }

    void LooseMolotovVisual::prepareResources() noexcept
    {
        resources.reset();
        try {
            if (!verifyNativeCalls()) {
                ROCK_LOG_ERROR(Hand, "Molotov flame unavailable: native model/clone/controller signature mismatch");
                return;
            }
            auto next = std::make_unique<Resources>();
            const std::uint64_t flags[2]{ 0, 0x2D };
            const auto load = reinterpret_cast<LoadModel>(REL::Module::get().base() + 0x1D0DEE0);
            // The native output already owns a reference. Pass the smart-pointer
            // storage directly rather than incrementing a raw owned result again.
            static_assert(sizeof(RE::NiPointer<RE::NiNode>) == sizeof(void*));
            if (load("Data/Meshes/Weapons/Grenade/MolotovCocktailProjectile.nif", &next->model, flags) != 0 || !next->model) {
                ROCK_LOG_ERROR(Hand, "Molotov flame unavailable: projectile model could not be loaded");
                return;
            }
            auto* wick = findShape(next->model.get(), "Wick002:0");
            if (!wick || wick->parent != next->model.get() ||
                !held_scene_presentation_policy::finiteTransform(wick->local) ||
                !loose_molotov_visual_policy::compatibleWick(wick->numVertices, wick->numTriangles, wick->modelBound, wick->modelBound)) {
                ROCK_LOG_WARN(Hand, "Molotov flame unavailable: projectile wick geometry or frame is unsupported");
                return;
            }
            next->wickLocal = wick->local;
            next->wickBound = wick->modelBound;
            for (std::size_t index = 0; index < next->shapes.size(); ++index) {
                auto* shape = findShape(next->model.get(), kShapeNames[index]);
                if (!shape || shape->parent != next->model.get() || shape->collisionObject || shape->skinInstance ||
                    !held_scene_presentation_policy::finiteTransform(shape->local) || !qualifyProperties(shape)) {
                    ROCK_LOG_WARN(Hand, "Molotov flame unavailable: source component rejected at shape={}", kShapeNames[index]);
                    return;
                }
                next->shapes[index].reset(shape);
            }
            resources = std::move(next);
            ROCK_LOG_INFO(Hand, "Molotov flame resources ready: shapes={} source=MolotovCocktailProjectile", kShapeNames.size());
        } catch (const std::exception& error) {
            ROCK_LOG_ERROR(Hand, "Molotov flame resource preparation failed: {}", error.what());
        } catch (...) {
            ROCK_LOG_ERROR(Hand, "Molotov flame resource preparation failed: unknown exception");
        }
    }

    std::unique_ptr<LooseMolotovVisual> LooseMolotovVisual::create(RE::TESObjectREFR* reference) noexcept
    {
        if (!reference) return {};
        const auto referenceId = reference->GetFormID();
        if (!resources) {
            ROCK_LOG_WARN(Hand, "Armed Molotov flame skipped: ref={:08X} reason=resources-unavailable", referenceId);
            return {};
        }
        try {
            auto visual = std::make_unique<LooseMolotovVisual>();
            visual->_referenceId = referenceId;
            visual->_referenceHandle = reference->GetHandle();
            return visual;
        } catch (...) {
            ROCK_LOG_ERROR(Hand, "Armed Molotov flame skipped: ref={:08X} reason=allocation-failed", referenceId);
            return {};
        }
    }

    LooseMolotovVisual::~LooseMolotovVisual() { release("armed-state-ended"); }

    bool LooseMolotovVisual::bind(RE::NiAVObject* root)
    {
        if (!resources) return false;
        auto* wick = findShape(root, "Wick:0");
        if (!wick || !wick->parent || !held_scene_presentation_policy::finiteTransform(wick->local) ||
            !loose_molotov_visual_policy::compatibleWick(wick->numVertices, wick->numTriangles, wick->modelBound, resources->wickBound)) {
            ROCK_LOG_WARN(Hand, "Armed Molotov flame skipped: ref={:08X} reason=unsupported-loose-wick", _referenceId);
            return false;
        }

        RE::NiCloningProcess cloning{};
        cloning.copyType = RE::NiCloningProcess::CopyType::kCopyExact;
        cloning.scale = { 1.0f, 1.0f, 1.0f };
        for (std::size_t index = 0; index < _parts.size(); ++index) {
            auto* source = resources->shapes[index].get();
            RE::NiPointer<RE::NiObject> copy{ cloneObject(source, &cloning) };
            auto* shape = copy ? copy->IsTriShape() : nullptr;
            if (!shape || shape == source || shape->parent || !qualifyProperties(shape, source)) {
                ROCK_LOG_WARN(Hand, "Armed Molotov flame skipped: ref={:08X} reason=clone-not-independent shape={}", _referenceId, kShapeNames[index]);
                detach("clone-rejected");
                return false;
            }
            auto& part = _parts[index];
            part.shape.reset(shape);
            shape->name = RE::BSFixedString(loose_molotov_visual_policy::kVisualShapeName);
            shape->local = loose_molotov_visual_policy::atWick(wick->local, resources->wickLocal, source->local);
            if (!held_scene_presentation_policy::finiteTransform(shape->local)) {
                detach("invalid-placement");
                ROCK_LOG_WARN(Hand, "Armed Molotov flame skipped: ref={:08X} reason=invalid-placement", _referenceId);
                return false;
            }
            for (std::size_t propertyIndex = 0; propertyIndex < 2; ++propertyIndex) {
                auto* property = shape->properties[propertyIndex].get();
                if (!property || !property->controllers) continue;
                // The cloned animated material is already private. Move its
                // controllers out of the scene's automatic update chain so only
                // this armed slot supplies time, including when Havok sleeps.
                part.controllers[propertyIndex] = std::move(property->controllers);
                for (auto* controller = part.controllers[propertyIndex].get(); controller; controller = controller->next.get())
                    startController(controller, 0.0f);
            }
        }
        _parent.reset(wick->parent);
        for (auto& part : _parts) {
            _parent->AttachChild(part.shape.get(), true);
            f4vr::updateDown(part.shape.get(), true);
        }
        ROCK_LOG_INFO(Hand, "Armed Molotov flame attached: ref={:08X} shapes={} wick={} elapsed={:.3f}s",
            _referenceId, _parts.size(), wick->name.c_str(), _elapsedSeconds);
        return true;
    }

    void LooseMolotovVisual::advanceAnimation()
    {
        RE::NiUpdateData update{};
        update.time = static_cast<float>(_elapsedSeconds);
        std::size_t updated = 0;
        for (auto& part : _parts) {
            for (auto& chain : part.controllers) {
                std::size_t count = 0;
                for (auto* controller = chain.get(); controller && count < kMaxControllersPerProperty;
                    controller = controller->next.get(), ++count) {
                    updateController(controller, &update);
                    ++updated;
                }
            }
        }
        if (!_reportedAnimation && _elapsedSeconds >= 0.5) {
            ROCK_LOG_INFO(Hand, "Armed Molotov flame animation serviced: ref={:08X} controllers={} elapsed={:.3f}s",
                _referenceId, updated, _elapsedSeconds);
            _reportedAnimation = true;
        }
    }

    void LooseMolotovVisual::update(RE::TESObjectREFR* reference, float deltaSeconds)
    {
        if (!reference || reference->GetFormID() != _referenceId || reference->IsDeleted() || reference->IsDisabled()) {
            release("reference-unavailable");
            return;
        }
        if (std::isfinite(deltaSeconds) && deltaSeconds > 0.0f) _elapsedSeconds += deltaSeconds;
        auto* root = reference->Get3D();
        if (root != _observedRoot.get()) {
            detach("reference-scene-changed");
            _observedRoot.reset(root);
            if (root) {
                try {
                    (void)bind(root);
                } catch (...) {
                    detach("attachment-exception");
                    ROCK_LOG_ERROR(Hand, "Armed Molotov flame skipped: ref={:08X} reason=attachment-exception", _referenceId);
                }
            }
        }
        if (_parent) {
            for (const auto& part : _parts) {
                if (!part.shape || part.shape->parent != _parent.get()) {
                    detach("attachment-replaced");
                    return;
                }
            }
            advanceAnimation();
        }
    }

    void LooseMolotovVisual::detach(const char* reason)
    {
        const bool attached = _parent != nullptr;
        // A player skeleton can change while this world bottle stays loaded.
        // The handle identifies its live scene for diagnostics. Retained parent
        // and shape references also permit removal from an unloaded/cached tree:
        // 141C17FB0 + 141C23E90 unlink Ni objects without touching a Havok world.
        // Leaving children on that tree would resurrect a frozen flame on reuse.
        const auto reference = _referenceHandle.get();
        const bool sameScene = reference && _observedRoot && reference->Get3D() == _observedRoot.get();
        for (auto& part : _parts) {
            if (_parent && part.shape && part.shape->parent == _parent.get()) {
                RE::NiPointer<RE::NiAVObject> detached;
                _parent->DetachChild(part.shape.get(), detached);
            }
            for (auto& chain : part.controllers) chain.reset();
            part.shape.reset();
        }
        _parent.reset();
        if (attached) ROCK_LOG_INFO(Hand, "Armed Molotov flame retired: ref={:08X} reason={} sceneAvailable={}",
            _referenceId, reason, sameScene);
    }

    void LooseMolotovVisual::release(const char* reason)
    {
        detach(reason);
        _observedRoot.reset();
    }
}
