#pragma once

#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"
#include "RE/Fallout.h"
#include "REL/Relocation.h"

#include <array>

namespace rock
{
    inline bool nativeSkinLayoutVerified()
    {
        // Native iterator 1402888E0 reads array live count at +10; resize
        // 141C358D0 distinguishes it from capacity at +8. Instance bones start +10.
        static const bool verified = [] {
            constexpr std::array<std::uint8_t, 6> expected{0x48, 0x8B, 0xC1, 0x8B, 0x49, 0x10};
            std::array<std::uint8_t, expected.size()> live{};
            const bool valid = native_memory::guardedCopyFromMemory(
                reinterpret_cast<const void*>(REL::Module::get().base() + 0x2888E0), live.data(), live.size()) && live == expected;
            if (!valid) ROCK_LOG_ERROR(MeshGrab, "Native skin count ABI mismatch; skin reads disabled");
            return valid;
        }();
        return verified;
    }

    inline RE::NiAVObject* resolveFlattenedSkinBoneOwner(RE::NiAVObject* skinRoot, const RE::NiTransform* worldTransform)
    {
        if (!skinRoot || !worldTransform || !native_memory::pointerRangeLooksReadable(skinRoot, sizeof(RE::NiAVObject))) return nullptr;
        // Native skin clone 141C34070 matches world pointers to tree+188,
        // entry stride A0 / world+40. 141C21030 and 141C20D30 read refNode+88;
        // 141C20D30 and 141C21690 establish the signed parent index at +80.
        static const bool verified = [] {
            constexpr std::array<std::uint8_t, 6> expected{0x45,0x33,0xC0,0x48,0x8B,0xC1};
            std::array<std::uint8_t, 6> live{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(REL::Module::get().base()+0x1C21030),
                live.data(),live.size()) && live==expected;
        }();
        if (!verified) return nullptr;
        auto resolveTree = [&](RE::NiAVObject* root) -> RE::NiAVObject* {
            std::uintptr_t vtable=0, entries=0;
            std::int32_t count=0;
            if (!native_memory::tryReadField(root,0,vtable) || vtable!=REL::Module::get().base()+0x2E580A8 ||
                !native_memory::tryReadField(root,0x180,count) || count<=0 || count>768 ||
                !native_memory::tryReadField(root,0x188,entries) || !entries) return nullptr;
            const auto pointer=reinterpret_cast<std::uintptr_t>(worldTransform);
            if (pointer<entries+0x40) return nullptr;
            const auto offset=pointer-entries-0x40;
            if (offset%0xA0 || offset/0xA0>=static_cast<std::size_t>(count)) return nullptr;
            auto index=static_cast<std::int32_t>(offset/0xA0);
            for (int depth=0; index>=0 && index<count && depth<count; ++depth) {
                const auto* entry=reinterpret_cast<const void*>(entries+std::size_t(index)*0xA0);
                RE::NiAVObject* owner=nullptr;
                std::int16_t parent=-1;
                if (!native_memory::tryReadField(entry,0x88,owner) || !native_memory::tryReadField(entry,0x80,parent)) return nullptr;
                if (owner) return native_memory::pointerRangeLooksReadable(owner,sizeof(RE::NiAVObject)) ? owner : nullptr;
                index=parent;
            }
            return nullptr;
        };
        if (auto* owner=resolveTree(skinRoot)) return owner;
        // 141C34070 calls the native tree search with depth one. Preserve that
        // scope rather than searching another actor or manufacturing bone nodes.
        if (auto* root=skinRoot->IsNode()) {
            auto& children=root->GetRuntimeData().children;
            if (children.size()>256) return nullptr;
            for (auto& child:children) if (child) if (auto* owner=resolveTree(child.get())) return owner;
        }
        return nullptr;
    }
}
