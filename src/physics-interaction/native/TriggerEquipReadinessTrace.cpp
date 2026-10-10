#include "physics-interaction/native/TriggerEquipReadinessTrace.h"

#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/weapon/EquippedWeaponVisualState.h"
#include "physics-interaction/PhysicsLog.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/InputEvent.h"
#include <Windows.h>
#include <atomic>
#include <chrono>
#include <string_view>

namespace rock::trigger_equip_readiness_trace
{
    namespace
    {
        using Clock = std::chrono::steady_clock;
        struct NativeSample
        {
            std::uint32_t form{}, stateBits{}, ammo{}, attackState{}, valid{};
            std::uintptr_t instance{}, data{}, fireNode{}, weaponPreload{}, projectilePreload{};
            std::uint8_t flags{};
            float cooldown{}, deadline{}, nativeNow{};
        };
        struct Capture
        {
            bool active{};
            std::uint64_t sequence{};
            std::uint32_t expectedForm{}, reference{}, samples{}, inputs{};
            std::uintptr_t boundInstance{};
            Clock::time_point started{}, lastFrame{}, lastInput{};
            NativeSample previous{};
            std::uint32_t previousVisual{};
        };
        // No engine pointers survive a callback. Other threads never access
        // the capture; the atomic owner check precedes every state access.
        std::atomic<DWORD> s_owner{};
        Capture s_capture{};

        bool onOwner() noexcept { return s_owner.load(std::memory_order_acquire) == GetCurrentThreadId(); }
        double elapsedMs() noexcept
        {
            return std::chrono::duration<double, std::milli>(Clock::now() - s_capture.started).count();
        }
        bool active() noexcept
        {
            if (!onOwner() || !s_capture.active) return false;
            if (!logger::isDebugEnabled()) { s_capture.active = false; return false; }
            if (elapsedMs() >= 5000.0) { stop("window-ended"); return false; }
            return true;
        }
        NativeSample sample() noexcept
        {
            NativeSample out{};
            auto* player = f4vr::getPlayer();
            if (!player) return out;
            // Verified FO4VR 1.2.72 preparation/update/AttackBlock fields.
            // +12A3:80 waits for +8A8; the native update derives bit 40 from
            // that wait and other native conditions. AttackBlock reads bit 40.
            if (native_memory::tryReadField(player, 0x134, out.stateBits)) out.valid |= 1;
            if (native_memory::tryReadField(player, 0x12A3, out.flags)) out.valid |= 2;
            if (native_memory::tryReadField(player, 0x908, out.cooldown)) out.valid |= 4;
            if (native_memory::tryReadField(player, 0x8A8, out.deadline)) out.valid |= 8;
            std::uint64_t ticks{};
            float secondsPerTick{};
            if (native_memory::tryReadValue(reinterpret_cast<const std::uint64_t*>(REL::Offset(0x5BBC058).address()), ticks) &&
                native_memory::tryReadValue(reinterpret_cast<const float*>(REL::Offset(0x2C81B8C).address()), secondsPerTick)) {
                out.nativeNow = static_cast<float>(ticks) * secondsPerTick;
                out.valid |= 16;
            }
            auto* item = f4vr::getEquippedWeaponItem();
            if (!item) return out;
            out.form = item->item.object->formID;
            out.instance = reinterpret_cast<std::uintptr_t>(item->item.instanceData.get());
            auto* data = item->data.get();
            out.data = reinterpret_cast<std::uintptr_t>(data);
            if (native_memory::tryReadField(data, offsetof(RE::EquippedWeaponData, ammoCount), out.ammo)) out.valid |= 32;
            if (native_memory::tryReadField(data, offsetof(RE::EquippedWeaponData, attackState), out.attackState)) out.valid |= 64;
            if (native_memory::tryReadField(data, offsetof(RE::EquippedWeaponData, fireNode), out.fireNode)) out.valid |= 128;
            if (native_memory::tryReadField(data, offsetof(RE::EquippedWeaponData, weaponPreload), out.weaponPreload)) out.valid |= 256;
            if (native_memory::tryReadField(data, offsetof(RE::EquippedWeaponData, projectilePreload), out.projectilePreload)) out.valid |= 512;
            return out;
        }
        void logNative(const char* event, const NativeSample& value) noexcept
        {
            ROCK_LOG_INFO(Weapon, "EQUIP-READY seq={} ms={:.2f} event={} expected={:08X} current={:08X} instance={:#x} data={:#x} stateBits={:08X} flags={:02X} presentationWait={} nativeAttackBlocked={} cooldown={:.4f} deadlineRemaining={:.4f} ammo={} attackState={} fireNode={:#x} weaponPreload={:#x} projectilePreload={:#x} valid={:03X}",
                s_capture.sequence, elapsedMs(), event, s_capture.expectedForm, value.form, value.instance, value.data,
                value.stateBits, value.flags, (value.flags & 0x80) != 0, (value.flags & 0x40) != 0,
                value.cooldown, value.deadline - value.nativeNow, value.ammo, value.attackState,
                value.fireNode, value.weaponPreload, value.projectilePreload, value.valid);
        }
    }

    void begin(std::uint32_t form, std::uint32_t reference) noexcept
    {
        if (!logger::isDebugEnabled()) return;
        DWORD unset = 0;
        s_owner.compare_exchange_strong(unset, GetCurrentThreadId(), std::memory_order_acq_rel);
        if (!onOwner()) return;
        if (s_capture.active) stop("next-trigger-equip");
        const auto next = s_capture.sequence + 1;
        s_capture = {};
        s_capture.active = true;
        s_capture.sequence = next;
        s_capture.expectedForm = form;
        s_capture.reference = reference;
        s_capture.started = Clock::now();
        ROCK_LOG_INFO(Weapon, "EQUIP-READY begin seq={} form={:08X} ref={:08X} thread={} windowMs=5000 maxFrames=64 maxInputEdges=32; preload pointers are presence witnesses, not completion flags", next, form, reference, GetCurrentThreadId());
        stage("before-inventory-scan");
    }
    void stage(const char* name) noexcept
    {
        if (active()) logNative(name, sample());
    }
    void frame(const equipped_weapon_visual_state::Snapshot& visual,
        bool bridgeVisible, bool gripPending, bool nativeAnimation, bool menu) noexcept
    {
        if (!active()) return;
        if (s_capture.samples >= 64) { stop("sample-budget"); return; }
        const auto value = sample();
        if (s_capture.boundInstance && (value.form != s_capture.expectedForm || value.instance != s_capture.boundInstance)) {
            stop("equipped-identity-changed"); return;
        }
        if (value.form == s_capture.expectedForm && value.instance) s_capture.boundInstance = value.instance;
        const std::uint32_t bits = (visual.weaponRoot ? 1u : 0u) | (visual.exactInstance ? 2u : 0u) |
            (visual.ancestorPathVisible ? 4u : 0u) | (visual.instanceLocallyVisible ? 8u : 0u) |
            (bridgeVisible ? 16u : 0u) | (gripPending ? 32u : 0u) | (nativeAnimation ? 64u : 0u) | (menu ? 128u : 0u);
        const auto& old = s_capture.previous;
        const bool changed = value.form != old.form || value.instance != old.instance || value.data != old.data ||
            value.stateBits != old.stateBits || value.flags != old.flags || value.ammo != old.ammo || value.attackState != old.attackState ||
            value.fireNode != old.fireNode || value.weaponPreload != old.weaponPreload || value.projectilePreload != old.projectilePreload ||
            value.valid != old.valid || (value.cooldown > 0) != (old.cooldown > 0) || bits != s_capture.previousVisual;
        const auto now = Clock::now();
        const double since = std::chrono::duration<double, std::milli>(now - s_capture.lastFrame).count();
        if (s_capture.samples && ((!changed && since < 100.0) || since < 25.0)) return;
        ++s_capture.samples;
        logNative("frame", value);
        ROCK_LOG_INFO(Weapon, "EQUIP-READY visual seq={} ms={:.2f} root={:#x} exact={:#x} ancestorsVisible={} localVisible={} bridge={} gripPending={} nativeAnimation={} menu={} ammoChanged={}",
            s_capture.sequence, elapsedMs(), reinterpret_cast<std::uintptr_t>(visual.weaponRoot), reinterpret_cast<std::uintptr_t>(visual.exactInstance),
            visual.ancestorPathVisible, visual.instanceLocallyVisible, bridgeVisible, gripPending, nativeAnimation, menu,
            (old.valid & 32) && (value.valid & 32) && old.instance == value.instance && old.ammo != value.ammo);
        s_capture.previous = value;
        s_capture.previousVisual = bits;
        s_capture.lastFrame = now;
    }
    void input(const RE::InputEvent* event, bool rockBlocked, bool nativeCalled, bool nativeAccepted) noexcept
    {
        if (!active() || !event || s_capture.inputs >= 32) return;
        const auto* button = event->As<RE::ButtonEvent>();
        const auto& name = event->QUserEvent();
        if (!button || std::string_view(name.c_str() ? name.c_str() : "", name.length()) != "WandTrigger" ||
            (button->QPressed() && !button->QJustPressed())) return;
        const auto now = Clock::now();
        if (std::chrono::duration<double, std::milli>(now - s_capture.lastInput).count() < 5.0) return;
        s_capture.lastInput = now;
        ++s_capture.inputs;
        ROCK_LOG_INFO(Input, "EQUIP-READY input seq={} ms={:.2f} pressed={} rockBlocked={} nativeCalled={} nativeAccepted={}",
            s_capture.sequence, elapsedMs(), button->QJustPressed(), rockBlocked, nativeCalled, nativeAccepted);
        logNative("input", sample());
    }
    void stop(const char* reason) noexcept
    {
        if (!onOwner() || !s_capture.active) return;
        ROCK_LOG_INFO(Weapon, "EQUIP-READY end seq={} ms={:.2f} reason={} frames={} inputEdges={}",
            s_capture.sequence, elapsedMs(), reason, s_capture.samples, s_capture.inputs);
        s_capture.active = false;
    }
}
