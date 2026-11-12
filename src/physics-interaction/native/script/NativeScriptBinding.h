#pragma once

/*
 * Read-only queries against the Papyrus scripts bound to a reference. Every
 * caller that needs to know which script a reference carries, or one of its
 * property values, goes through this one validated path to the game's virtual
 * machine. Game thread only; the returned script pointer is a counted VM
 * reference that the caller drops before the frame ends.
 */

#include "RE/Bethesda/BSScript/Object.h"
#include "RE/Bethesda/BSTSmartPointer.h"

namespace RE
{
    class TESObjectREFR;

    namespace BSScript
    {
        class IVirtualMachine;
    }
}

namespace rock::native_script_binding
{
    struct BoundScript
    {
        RE::BSScript::IVirtualMachine* vm = nullptr;
        RE::BSTSmartPointer<RE::BSScript::Object> script{};
        // Set when the query could not be answered (VM, handle or script object
        // failed validation). Null with an empty script means "not bound".
        const char* failedStage = nullptr;
    };

    // Exact type-name match, so a script extending scriptName does not count.
    [[nodiscard]] BoundScript findBoundScript(RE::TESObjectREFR* ref, const char* scriptName);

    // Reads a bool property of a bound script. Returns false and names the
    // failed stage when the type, its property table or the value is unusable.
    [[nodiscard]] bool readBoolProperty(const BoundScript& bound, const char* propertyName, bool& value,
        const char*& failedStage);
}
