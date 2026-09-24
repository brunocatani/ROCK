#include "physics-interaction/native/DecorationPlacement.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/PhysicsRecursiveWrappers.h"
#include "physics-interaction/native/HavokWorldLock.h"
#include "physics-interaction/object/PhysicsBodyClassifier.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/PhysicsLog.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/Havok/hknpBody.h"
#include "RE/Havok/hkVector4.h"
#include "RE/NetImmerse/NiAVObject.h"
#include "RE/Bethesda/GameScript.h"
#include "RE/Bethesda/BSScript/Object.h"
#include "RE/Bethesda/BSScript/IVirtualMachine.h"
#include "RE/Bethesda/BSScript/IObjectHandlePolicy.h"
#include <array>
#include <cmath>

namespace rock::decoration_placement {
namespace {
    template<class Fn, std::size_t N>
    Fn checkedEntry(std::uintptr_t rva, const std::array<std::uint8_t,N>& expected) {
        const auto address=REL::Offset(rva).address();
        const auto segment=REL::Module::get().segment(REL::Segment::text);
        std::array<std::uint8_t,N> actual{};
        if (!REL::Module::IsVR() || address<segment.address() ||
            address+N>segment.address()+segment.size() ||
            !native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(address),actual.data(),N) || actual!=expected) {
            ROCK_LOG_ERROR(Init,"Decoration native entry rejected rva={:X}",rva);
            return nullptr;
        }
        return reinterpret_cast<Fn>(address);
    }
    using PoseSetter=void(*)(RE::TESObjectREFR*,const RE::NiPoint3&);
    using WorldLock=void(*)(RE::hknpWorld*);
    struct Native {
        // Raw VR setter witnesses: 1403F4370/1403F4080, and callers
        // 141483930/1403F7210. Both setters mark CHANGE_REFR_MOVE (2).
        PoseSetter position=checkedEntry<PoseSetter>(0x3F4370,std::array<std::uint8_t,6>{0x48,0x89,0x5C,0x24,0x18,0x56});
        PoseSetter rotation=checkedEntry<PoseSetter>(0x3F4080,std::array<std::uint8_t,6>{0x48,0x89,0x5C,0x24,0x08,0x57});
        // Papyrus SetMotionType's native worker 1414B22B0 holds this
        // hknpWorld write lock around the recursive motion change and AddChange(4).
        WorldLock lock=checkedEntry<WorldLock>(0x1DF5FB0,std::array<std::uint8_t,7>{0x48,0x81,0xC1,0xD8,0x06,0x00,0x00});
        WorldLock unlock=checkedEntry<WorldLock>(0x1DF5FD0,std::array<std::uint8_t,7>{0x48,0x81,0xC1,0xD8,0x06,0x00,0x00});
        bool valid() const { return position && rotation && lock && unlock; }
    };
    const Native& native() { static const Native value; return value; }
    struct WriteScope {
        RE::hknpWorld* world;
        explicit WriteScope(RE::hknpWorld* value):world(value) { native().lock(world); }
        ~WriteScope() { native().unlock(world); }
    };
}
bool available() { return native().valid(); }
ScriptPreparation prepareLoadScript(RE::TESObjectREFR* ref, bool start) {
    // Vanilla DefaultDisableHavokOnLoad otherwise calls MoveToMyEditorLocation
    // on reload. Mark this deliberately placed reference as already simulated,
    // through the engine's property setter, never by editing VM variable memory.
    // GameVM+0xB0 and VM slots 23/24 are witnessed by 1413FA6E0/1413FA890;
    // object type +8 and variable readback are witnessed by 142703060/142703C60.
    auto reject=[&](const char* stage) {
        ROCK_LOG_WARN(Hand,"Decoration rejected ref={:08X} stage={}",ref ? ref->GetFormID() : 0,stage);
        return ScriptPreparation::Rejected;
    };
    if (!ref) return reject("script-reference");
    RE::GameVM* gameVm=nullptr;
    if (!native_memory::tryReadValue(reinterpret_cast<RE::GameVM**>(REL::Offset(0x5935428).address()),gameVm) ||
        !native_memory::pointerRangeLooksReadable(gameVm,0xB8)) return reject("game-vm");
    RE::BSScript::IVirtualMachine* vm=nullptr;
    std::uintptr_t vtable=0;
    if (!native_memory::tryReadField(gameVm,0xB0,vm) ||
        !native_memory::tryReadField(vm,0,vtable) || vtable!=REL::Offset(0x309D998).address()) return reject("script-vm-identity");
    auto& handles=vm->GetObjectHandlePolicy();
    if (!native_memory::pointerRangeLooksReadable(&handles,sizeof(void*))) return reject("script-handle-policy");
    const auto handle=handles.GetHandleForObject(static_cast<std::uint32_t>(ref->GetFormType()),ref);
    if (handle==handles.EmptyHandle()) return reject("script-handle");
    RE::BSTSmartPointer<RE::BSScript::Object> script;
    if (!vm->FindBoundObject(handle,"DefaultDisableHavokOnLoad",false,script,true)) return ScriptPreparation::Ready;
    if (!script || !native_memory::pointerRangeLooksReadable(script.get(),sizeof(RE::BSScript::Object))) return reject("load-script");
    auto* type=script->GetTypeInfo();
    if (!type || !native_memory::pointerRangeLooksReadable(type,sizeof(*type)) || !type->Valid() ||
        type->GetNumProperties()>64) return reject("load-script-type");
    // 1426C2060 derives this span from type +40/+44/+50; 1426C5FC0
    // independently confirms the linked state, variable count and data base.
    // The property search at 1426C6AC0 advances in 0x48-byte records.
    const auto* properties=type->GetPropertyIter();
    if (!type->data || !type->GetNumProperties() ||
        !native_memory::pointerRangeLooksReadable(properties,
            type->GetNumProperties()*sizeof(*properties))) return reject("load-script-property-span");
    const auto index=type->GetPropertyIndex(RE::BSFixedString("BeenSimmed"));
    RE::BSScript::Variable value;
    if (index==0xFFFF'FFFFu || !vm->GetVariableValue(script,index,value) || !value.is<bool>()) return reject("load-script-property");
    if (RE::BSScript::get<bool>(value)) return ScriptPreparation::Ready;
    if (start) {
        RE::BSScript::Variable simulated;
        simulated=true;
        if (!vm->SetPropertyValue(script,"BeenSimmed",simulated,{})) return reject("load-script-setter");
    }
    // The VM may execute its setter later. The game-thread caller keeps the
    // grab intact and reads back completion on a later frame, with a deadline.
    return ScriptPreparation::Pending;
}
bool anchor(RE::TESObjectREFR* ref, RE::hknpWorld* world,
    const RE::NiTransform& pose, std::span<const std::uint32_t> bodyIds) {
    if (!available() || !ref || ref->IsDeleted() || ref->IsDisabled() || !world ||
        bodyIds.empty() || bodyIds.size()>64 || !ref->Get3D() ||
        !std::isfinite(pose.translate.x) || !std::isfinite(pose.translate.y) || !std::isfinite(pose.translate.z)) return false;
    const auto angles=transform_math::matrixToReferenceEulerRadians<RE::NiMatrix3,RE::NiPoint3>(pose.rotate);
    if (!std::isfinite(angles.x) || !std::isfinite(angles.y) || !std::isfinite(angles.z)) return false;
    auto* root=ref->Get3D();
    WriteScope lock(world);
    const RE::hkVector4f zero{};
    for (const auto id:bodyIds) {
        if (!havok_runtime::setBodyVelocityDeferred(world,id,zero,zero)) {
            ROCK_LOG_WARN(Hand,"Decoration rejected ref={:08X} stage=stop-velocity body={}",ref->GetFormID(),id);
            return false;
        }
    }
    using physics_recursive_wrappers::MotionPreset;
    bool frozen=physics_recursive_wrappers::setMotionRecursive(root,MotionPreset::Keyframed,true,false,false);
    for (const auto id:bodyIds) {
        const auto body=havok_runtime::snapshotBody(world,RE::hknpBodyId{id});
        frozen=frozen && body.valid && body.body &&
            physics_body_classifier::motionTypeFromBodyFlags(body.body->flags)==physics_body_classifier::BodyMotionType::Keyframed;
    }
    if (!frozen) {
        const bool restored=physics_recursive_wrappers::setMotionRecursive(root,MotionPreset::Dynamic,true,true,true);
        ROCK_LOG_ERROR(Hand,"Decoration freeze rejected ref={:08X} stage=motion-verification restoredDynamic={}",ref->GetFormID(),restored);
        ref->AddChange(4);
        return false;
    }
    // Matches native SetMotionType worker 1414B23B2..1414B23BD. Native
    // SaveHavokData writes the keyframe bit (1403D8CEA); LoadHavokData
    // restores preset 2 from that bit (1403D982F..1403D983C).
    native().position(ref,pose.translate);
    native().rotation(ref,angles);
    ref->AddChange(4);
    return true;
}
}
