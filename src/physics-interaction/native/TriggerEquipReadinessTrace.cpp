#include "physics-interaction/native/TriggerEquipReadinessTrace.h"

#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/weapon/EquippedWeaponVisualState.h"
#include "physics-interaction/weapon/WeaponScenePath.h"
#include "physics-interaction/TransformMath.h"
#include "RE/NetImmerse/NiNode.h"
#include "physics-interaction/PhysicsLog.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/InputEvent.h"
#include <Windows.h>
#include <atomic>
#include <chrono>
#include <array>
#include <cstring>
#include <algorithm>
#include <cmath>
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
            std::uint32_t requests{}, actions{};
            std::uintptr_t boundInstance{};
            Clock::time_point started{}, lastFrame{}, lastInput{};
            NativeSample previous{};
            std::uint32_t previousVisual{};
        };
        // No engine pointers survive a callback. Other threads never access
        // the capture; the atomic owner check precedes every state access.
        std::atomic<DWORD> s_owner{};
        Capture s_capture{};
        struct PartRecord
        {
            weapon_scene::Path<RE::NiAVObject,32> path;
            std::array<char,96> name{};
        };
        struct PartSet
        {
            std::uintptr_t root{};
            std::array<PartRecord,24> nodes{};
            std::size_t count{};
            unsigned samples{}, discoveries{};
            Clock::time_point last{};
        };
        // Loose, pre-handling native, final native, retained bridge. Paths store identities and
        // slots only, resolving from each callback's live root before reads.
        std::array<PartSet,4> s_parts{};
        using AttackRequest = void (*)(void*);
        using DispatchAction = bool (*)(void*, std::int32_t, std::uint32_t);
        AttackRequest s_request{};
        DispatchAction s_dispatch{};
        bool s_actionHookInstalled{};

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
        void logHandler(const char* phase, const void* handler) noexcept
        {
            // Verified FC7BA0 and FCB100 gate/request fields; read only.
            float holdoff{};
            std::uint32_t mode{}, flags{};
            const bool valid = native_memory::tryReadField(handler,0x50,holdoff) &&
                native_memory::tryReadField(handler,0x38,mode) &&
                native_memory::tryReadField(handler,0x74,flags);
            ROCK_LOG_INFO(Input, "EQUIP-READY handler seq={} ms={:.2f} phase={} readable={} holdoff={:.4f} mode={} requestFlags={:08X} requestPending={} requestLatched={}",
                s_capture.sequence,elapsedMs(),phase,valid,holdoff,mode,flags,flags&255,(flags>>8)&255);
        }
        void attackRequest(void* handler)
        {
            const bool trace = active() && s_capture.requests < 32;
            if (trace) { ++s_capture.requests; logHandler("request-enter",handler); }
            s_request(handler);
            if (trace) { logHandler("request-return",handler); logNative("attack-request-return",sample()); }
        }
        bool dispatchAction(void* controls, std::int32_t action, std::uint32_t priority)
        {
            const bool trace = active() && s_capture.actions < 32;
            const auto start = trace ? Clock::now() : Clock::time_point{};
            const bool accepted = s_dispatch(controls,action,priority);
            if (trace) {
                ++s_capture.actions;
                ROCK_LOG_INFO(Input, "EQUIP-READY action seq={} ms={:.2f} action={} priority={} accepted={} dispatchMs={:.3f}",
                    s_capture.sequence,elapsedMs(),action,priority,accepted,
                    std::chrono::duration<double,std::milli>(Clock::now()-start).count());
                logNative("action-return",sample());
            }
            return accepted;
        }
    }

    void install() noexcept
    {
        // Installation runs at F4SE Load, before the user's logging config is
        // applied. Only capture/output may use that setting; gating installation
        // here silently omitted both downstream witnesses in the prior build.
        if (s_request || !REL::Module::IsVR() ||
            REL::Module::get().version()!=F4SE::RUNTIME_VR_1_2_72) return;
        constexpr std::array<std::uint8_t,14> prefix{0x40,0x53,0x48,0x83,0xEC,0x40,0x0F,0x29,0x74,0x24,0x30,0x0F,0x57,0xF6};
        void* original = nullptr;
        if (!entry_trampoline_hook::install("Trigger equip native attack request observation",0xFC7BA0,prefix.data(),prefix.size(),reinterpret_cast<void*>(&attackRequest),original)) return;
        s_request = reinterpret_cast<AttackRequest>(original);
        constexpr std::array<std::uint8_t,5> expected{0xE8,0xF7,0x8B,0xFF,0xFF};
        std::array<std::uint8_t,5> actual{};
        const auto call = REL::Offset(0xFC7BE4).address();
        if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(call),actual.data(),actual.size()) || actual!=expected || F4SE::GetTrampoline().free_size()<14) {
            ROCK_LOG_WARN(Init,"Trigger equip action-result observation unavailable; request observation retained"); return;
        }
        s_dispatch = reinterpret_cast<DispatchAction>(REL::Offset(0xFC07E0).address());
        try {
            (void)F4SE::GetTrampoline().write_call<5>(call,&dispatchAction);
            FlushInstructionCache(GetCurrentProcess(),reinterpret_cast<const void*>(call),5);
            s_actionHookInstalled = true;
            ROCK_LOG_INFO(Init,"Trigger equip native request and action-result observation installed; no attack behavior changed");
        } catch (...) {
            s_actionHookInstalled = false;
            REL::safe_write(call,expected.data(),expected.size());
            FlushInstructionCache(GetCurrentProcess(),reinterpret_cast<const void*>(call),5);
            ROCK_LOG_WARN(Init,"Trigger equip action-result observation failed; native action call retained");
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
        s_parts = {};
        s_capture.active = true;
        s_capture.sequence = next;
        s_capture.expectedForm = form;
        s_capture.reference = reference;
        s_capture.started = Clock::now();
        ROCK_LOG_INFO(Weapon, "EQUIP-READY begin seq={} form={:08X} ref={:08X} thread={} requestHook={} actionHook={} windowMs=5000 maxFrames=64 maxInputEdges=32; preload pointers are presence witnesses, not completion flags", next, form, reference, GetCurrentThreadId(), s_request != nullptr, s_actionHookInstalled);
        stage("before-inventory-scan");
    }
    void stage(const char* name) noexcept
    {
        if (active()) logNative(name, sample());
    }
    bool partCaptureActive() noexcept
    {
        return active() && elapsedMs() < 1500.0;
    }
    void parts(RE::NiAVObject* root, unsigned phase, bool bridgeVisible) noexcept
    {
        if (!partCaptureActive() || !root || phase >= s_parts.size()) return;
        auto& set = s_parts[phase];
        const auto now = Clock::now();
        if (set.samples >= 48 || (set.samples &&
            std::chrono::duration<double,std::milli>(now-set.last).count() < 30.0)) return;
        if (set.root != reinterpret_cast<std::uintptr_t>(root)) {
            if (set.discoveries >= 4) return;
            ++set.discoveries;
            set.root = reinterpret_cast<std::uintptr_t>(root);
            set.count = 0;
            std::array<RE::NiAVObject*,512> todo{};
            std::size_t pending = 1, visited = 0;
            todo[0] = root;
            while (pending && visited++ < todo.size()) {
                auto* object = todo[--pending];
                auto* branch = object ? object->IsNode() : nullptr;
                if (!branch) continue;
                const auto* text = object->name.c_str();
                std::array<char,96> lower{};
                if (text) {
                    const auto length = (std::min)(static_cast<std::size_t>(object->name.length()),lower.size()-1);
                    for (std::size_t i=0;i<length;++i) lower[i] = text[i]>='A'&&text[i]<='Z'?text[i]+('a'-'A'):text[i];
                    const std::string_view name(lower.data(),length);
                    const bool part = name.find("child")==std::string_view::npos &&
                        (name.find("mag")!=std::string_view::npos || name.find("bolt")!=std::string_view::npos ||
                         name.find("slide")!=std::string_view::npos || name.find("selector")!=std::string_view::npos ||
                         name.find("safety")!=std::string_view::npos || name.find("charging")!=std::string_view::npos ||
                         name.find("weaponextra")!=std::string_view::npos || name.find("trigger")!=std::string_view::npos);
                    if (part && set.count < set.nodes.size()) {
                        auto& record = set.nodes[set.count];
                        if (record.path.capture(root,object)) {
                            record.name = {};
                            std::copy_n(text,length,record.name.data());
                            ++set.count;
                        }
                    }
                }
                // Preserve child order; bound malformed/oversized hierarchies.
                const auto slots = branch->children.capacity();
                if (slots > 128) continue;
                for (std::size_t i=slots;i>0 && pending<todo.size();--i) {
                    auto* child=branch->children[static_cast<decltype(slots)>(i-1)].get();
                    if (child) todo[pending++]=child;
                }
            }
            ROCK_LOG_INFO(Weapon,"EQUIP-PART discovery seq={} phase={} root={:#x} nodes={} visited={} pending={} truncated={}",
                s_capture.sequence,phase,set.root,set.count,visited,pending,pending!=0||set.count==set.nodes.size());
        }
        const auto rootWorld = root->world;
        if (!std::isfinite(rootWorld.scale) || std::abs(rootWorld.scale) < 0.00001f) return;
        const auto inverse = transform_math::invertTransform(rootWorld);
        ++set.samples;
        set.last = now;
        ROCK_LOG_INFO(Weapon,"EQUIP-PART frame seq={} ms={:.2f} phase={} sample={} bridge={} root={:#x} rootWorld=({:.4f},{:.4f},{:.4f}) scale={:.5f} culled={}",
            s_capture.sequence,elapsedMs(),phase,set.samples,bridgeVisible,set.root,
            rootWorld.translate.x,rootWorld.translate.y,rootWorld.translate.z,rootWorld.scale,root->GetAppCulled());
        for (std::size_t i=0;i<set.count;++i) {
            const auto& record=set.nodes[i];
            auto* node=record.path.resolve(root);
            if (!node) {
                ROCK_LOG_INFO(Weapon,"EQUIP-PART missing seq={} phase={} name='{}' root={:#x}",s_capture.sequence,phase,record.name.data(),set.root);
                set.root=0; // Rediscover next sample; never dereference old nodes.
                continue;
            }
            const auto& local=node->local;
            const auto relative=transform_math::composeTransforms(inverse,node->world);
            ROCK_LOG_INFO(Weapon,"EQUIP-PART pose seq={} phase={} sample={} name='{}' node={:#x} parent={:#x} culled={} localT=({:.4f},{:.4f},{:.4f}) localS={:.5f} localR=({:.4f},{:.4f},{:.4f};{:.4f},{:.4f},{:.4f};{:.4f},{:.4f},{:.4f}) relativeT=({:.4f},{:.4f},{:.4f}) relativeS={:.5f} relativeR=({:.4f},{:.4f},{:.4f};{:.4f},{:.4f},{:.4f};{:.4f},{:.4f},{:.4f})",
                s_capture.sequence,phase,set.samples,record.name.data(),reinterpret_cast<std::uintptr_t>(node),reinterpret_cast<std::uintptr_t>(node->parent),node->GetAppCulled(),
                local.translate.x,local.translate.y,local.translate.z,local.scale,
                local.rotate.entry[0][0],local.rotate.entry[0][1],local.rotate.entry[0][2],local.rotate.entry[1][0],local.rotate.entry[1][1],local.rotate.entry[1][2],local.rotate.entry[2][0],local.rotate.entry[2][1],local.rotate.entry[2][2],
                relative.translate.x,relative.translate.y,relative.translate.z,relative.scale,
                relative.rotate.entry[0][0],relative.rotate.entry[0][1],relative.rotate.entry[0][2],relative.rotate.entry[1][0],relative.rotate.entry[1][1],relative.rotate.entry[1][2],relative.rotate.entry[2][0],relative.rotate.entry[2][1],relative.rotate.entry[2][2]);
        }
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
    void input(const RE::InputEvent* event, bool rockBlocked, bool nativeCalled, bool nativeAccepted, const void* handler) noexcept
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
        if (handler) logHandler("input-gate",handler);
    }
    void stop(const char* reason) noexcept
    {
        if (!onOwner() || !s_capture.active) return;
        ROCK_LOG_INFO(Weapon, "EQUIP-READY end seq={} ms={:.2f} reason={} frames={} inputEdges={}",
            s_capture.sequence, elapsedMs(), reason, s_capture.samples, s_capture.inputs);
        s_capture.active = false;
    }
}
