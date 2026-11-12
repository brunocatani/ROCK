#include "physics-interaction/native/script/NativeScriptBinding.h"

#include "RE/Bethesda/BSScript/IObjectHandlePolicy.h"
#include "RE/Bethesda/BSScript/IVirtualMachine.h"
#include "RE/Bethesda/GameScript.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "physics-interaction/native/NativeMemory.h"

namespace rock::native_script_binding
{
    namespace
    {
        // GameVM+0xB0 and VM slots 23/24 are witnessed by 1413FA6E0/1413FA890;
        // the VM vtable identity gates every call through it.
        constexpr std::uintptr_t kGameVmSingletonRva = 0x5935428;
        constexpr std::ptrdiff_t kGameVmVirtualMachineOffset = 0xB0;
        constexpr std::uintptr_t kVirtualMachineVtableRva = 0x309D998;
        constexpr std::uint32_t kMaxPlausibleProperties = 64;

        RE::BSScript::IVirtualMachine* resolveVirtualMachine(const char*& failedStage)
        {
            RE::GameVM* gameVm = nullptr;
            if (!native_memory::tryReadValue(reinterpret_cast<RE::GameVM**>(REL::Offset(kGameVmSingletonRva).address()),
                    gameVm) ||
                !native_memory::pointerRangeLooksReadable(gameVm, kGameVmVirtualMachineOffset + sizeof(void*))) {
                failedStage = "game-vm";
                return nullptr;
            }
            RE::BSScript::IVirtualMachine* vm = nullptr;
            std::uintptr_t vtable = 0;
            if (!native_memory::tryReadField(gameVm, kGameVmVirtualMachineOffset, vm) ||
                !native_memory::tryReadField(vm, 0, vtable) ||
                vtable != REL::Offset(kVirtualMachineVtableRva).address()) {
                failedStage = "script-vm-identity";
                return nullptr;
            }
            return vm;
        }
    }

    BoundScript findBoundScript(RE::TESObjectREFR* ref, const char* scriptName)
    {
        BoundScript result{};
        if (!ref || !scriptName) {
            result.failedStage = "script-reference";
            return result;
        }
        auto* vm = resolveVirtualMachine(result.failedStage);
        if (!vm)
            return result;
        auto& handles = vm->GetObjectHandlePolicy();
        if (!native_memory::pointerRangeLooksReadable(&handles, sizeof(void*))) {
            result.failedStage = "script-handle-policy";
            return result;
        }
        const auto handle = handles.GetHandleForObject(static_cast<std::uint32_t>(ref->GetFormType()), ref);
        if (handle == handles.EmptyHandle()) {
            result.failedStage = "script-handle";
            return result;
        }
        result.vm = vm;
        if (!vm->FindBoundObject(handle, scriptName, false, result.script, true)) {
            result.script.reset();
            return result;
        }
        if (!result.script ||
            !native_memory::pointerRangeLooksReadable(result.script.get(), sizeof(RE::BSScript::Object))) {
            result.script.reset();
            result.failedStage = "script-object";
        }
        return result;
    }

    bool readBoolProperty(const BoundScript& bound, const char* propertyName, bool& value, const char*& failedStage)
    {
        if (!bound.vm || !bound.script || !propertyName) {
            failedStage = "script-object";
            return false;
        }
        auto* type = bound.script->GetTypeInfo();
        if (!type || !native_memory::pointerRangeLooksReadable(type, sizeof(*type)) || !type->Valid() ||
            type->GetNumProperties() > kMaxPlausibleProperties) {
            failedStage = "script-type";
            return false;
        }
        // 1426C2060 derives this span from type +40/+44/+50; 1426C5FC0
        // independently confirms the linked state, variable count and data base.
        // The property search at 1426C6AC0 advances in 0x48-byte records.
        const auto* properties = type->GetPropertyIter();
        if (!type->data || !type->GetNumProperties() ||
            !native_memory::pointerRangeLooksReadable(properties, type->GetNumProperties() * sizeof(*properties))) {
            failedStage = "script-property-span";
            return false;
        }
        const auto index = type->GetPropertyIndex(RE::BSFixedString(propertyName));
        RE::BSScript::Variable variable;
        if (index == 0xFFFF'FFFFu || !bound.vm->GetVariableValue(bound.script, index, variable) ||
            !variable.is<bool>()) {
            failedStage = "script-property";
            return false;
        }
        value = RE::BSScript::get<bool>(variable);
        return true;
    }
}
