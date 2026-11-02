#include "physics-interaction/native/NativeQuickLootPanel.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/native/NativeNiNodeFactory.h"
#include "rock_support/Fo4VrRuntime.h"

namespace rock::native_quick_loot_panel
{
    namespace
    {
        /*
         * FO4VR 1.2.72, raw disassembly of the HUDQuickContainerDataModel
         * update 0xA92290. It copies the right and left wand picks from the
         * wand view-caster events into model +0x364 (0xA923E4) and +0x368
         * (0xA923F7), then locks model +0x268 (lea 0xA923FD, call 0xA92417 to
         * the spin-lock acquire 0x24E10). Everything after that reads the
         * picks from the two slots: the target choice 0xA96150 returns +0x364
         * when 0xA94EA0 accepts it, else +0x368, and the data the panel
         * renders records whether +0x364 is lootable (0xA928C0..0xA928EE).
         * The panel lists a container only when that primary flag is set, so
         * a pick by the left wand alone chose the container but left the
         * panel blank.
         *
         * The lock call is redirected. Before it runs, when only the left pick
         * is lootable the two slots are swapped, so the model treats it as the
         * primary pick for the rest of this update and until the next update
         * copies the real picks again. 0xF0A130, which gates the copy, returns
         * true on VR.
         */
        constexpr std::uintptr_t kPicksCopiedLockCall = 0xA92417;
        constexpr std::uintptr_t kSpinLockAcquire = 0x24E10;
        constexpr std::ptrdiff_t kModelLockOffset = 0x268;
        constexpr std::ptrdiff_t kRightPickOffset = 0x364;
        constexpr std::ptrdiff_t kLeftPickOffset = 0x368;
        constexpr std::uintptr_t kIsLootableTarget = 0xA94EA0;
        // The engine's invalid reference handle, compared at 0xA96186.
        constexpr std::uintptr_t kInvalidHandle = 0x392E884;

        struct ExpectedBytes
        {
            std::uintptr_t address;
            std::array<std::uint8_t, 7> bytes;
            std::size_t size;
        };

        constexpr std::array<ExpectedBytes, 4> kExpectedPickCopy{ {
            { 0xA923E4, { 0x89, 0x86, 0x64, 0x03, 0x00, 0x00 }, 6 }, // mov [rsi+0x364], eax
            { 0xA923F7, { 0x89, 0x86, 0x68, 0x03, 0x00, 0x00 }, 6 }, // mov [rsi+0x368], eax
            { 0xA923FD, { 0x48, 0x8D, 0x9E, 0x68, 0x02, 0x00, 0x00 }, 7 }, // lea rbx, [rsi+0x268]
            { 0xA9240D, { 0x48, 0x8B, 0xCB }, 3 }, // mov rcx, rbx
        } };

        /*
         * HUDRolloverModel vtable slot 16 (call [vtable+0x80]) says whether a
         * wand's Search prompt hosts the quick container: the primary model
         * returns true (0xADDD60, mov al,1), the secondary false (0xADDD80,
         * xor al,al). Its only callers in the rollover code (0xAC7870,
         * 0xAC7E10, 0xAC7F60, 0xAD8930) pair it with the quick container
         * state's primary-lootable flag (0xAC5C30 reads state +0xC0) to hide
         * the hosting wand's prompt while the panel lists, and with dead-actor
         * picks. Both slots are replaced so a wand hosts the panel exactly when
         * it points at the listed container.
         */
        struct RolloverHostSlot
        {
            std::uintptr_t vtable;
            std::uintptr_t native;
            std::array<std::uint8_t, 3> nativeBytes;
        };

        constexpr std::size_t kHostsQuickContainerSlot = 16;
        constexpr RolloverHostSlot kPrimaryRolloverHost{ 0x2D4ED78, 0xADDD60, { 0xB0, 0x01, 0xC3 } };
        constexpr RolloverHostSlot kSecondaryRolloverHost{ 0x2D4EF98, 0xADDD80, { 0x32, 0xC0, 0xC3 } };

        /*
         * WSLootModel: its constructor 0xC85560 stores itself at 0x5ACBC98.
         * The player VR node builder 0xEF21A0 attaches it to
         * PrimaryUIAttachNode (player +0x700) through 0xC90220, which attaches
         * model +0x10; the model's placement 0xC85680 writes the fHUDLoot
         * offset, pitch and scale into that same root's local transform.
         */
        constexpr std::uintptr_t kLootModel = 0x5ACBC98;
        constexpr std::ptrdiff_t kLootModelRootOffset = 0x10;

        /*
         * The panel reads at its hand size when the eye is this far from it
         * (about 60 cm, a hand held up to read). Further away it is scaled up
         * with distance so it keeps that apparent size, up to kMaxViewScale.
         * Game units.
         */
        constexpr float kHandReadingDistance = 45.0f;
        constexpr float kMaxViewScale = 4.0f;
        // The panel stands at most this fraction of the way from the
        // container's centre to the player, so a large bound cannot bring it
        // into the player's face.
        constexpr float kMaxApproachFraction = 0.5f;

        using SpinLockAcquire = void* (*)(void*, std::uintptr_t);
        using IsLootableTarget = bool (*)(void*, std::uint32_t*);

        // Which wands point at the container the panel lists.
        enum ListedFlags : std::uint8_t
        {
            kListed = 1 << 0,
            kRightPicksListed = 1 << 1,
            kLeftPicksListed = 1 << 2,
        };

        SpinLockAcquire g_lockAcquire{ nullptr };
        bool g_attempted{ false };
        // Written by the HUD data model update; read by the rollover models
        // and the game thread.
        std::atomic<std::uint8_t> g_listedFlags{ 0 };
        std::atomic<std::uint32_t> g_listedPick{ 0 };
        // Game thread only. The ROCK node under the room node that carries the
        // panel at its container; it outlives player node rebuilds and is
        // re-parented to the current room node when the old one is gone.
        RE::NiPointer<RE::NiNode> g_carrier;
        bool g_carrierFailed{ false };
        bool g_onContainerLogged{ false };

        // Runs once per model update, right after the real picks are copied.
        void chooseListedPick(std::byte* model)
        {
            static REL::Relocation<IsLootableTarget> isLootable{ REL::Offset(kIsLootableTarget) };
            const std::uint32_t invalid = *REL::Relocation<const std::uint32_t*>{ REL::Offset(kInvalidHandle) };
            auto* const rightSlot = reinterpret_cast<std::uint32_t*>(model + kRightPickOffset);
            auto* const leftSlot = reinterpret_cast<std::uint32_t*>(model + kLeftPickOffset);
            const std::uint32_t right = *rightSlot;
            const std::uint32_t left = *leftSlot;

            // 0xA94EA0 takes the handle by address; the native callers pass a copy.
            std::uint32_t candidate = right;
            const bool rightLootable = right != invalid && isLootable(model, &candidate);
            candidate = left;
            const bool leftLootable = left != invalid && isLootable(model, &candidate);
            if (!rightLootable && !leftLootable) {
                g_listedFlags.store(0, std::memory_order_relaxed);
                g_listedPick.store(0, std::memory_order_release);
                return;
            }

            // The native choice: the right pick when lootable, else the left one.
            const std::uint32_t listed = rightLootable ? right : left;
            if (!rightLootable) {
                *rightSlot = left;
                *leftSlot = right;
            }
            g_listedFlags.store(static_cast<std::uint8_t>(kListed | (right == listed ? kRightPicksListed : 0) |
                                    (left == listed ? kLeftPicksListed : 0)),
                std::memory_order_relaxed);
            g_listedPick.store(listed, std::memory_order_release);
        }

        void* afterPicksCopied(void* lock, const std::uintptr_t argument)
        {
            // The redirected call passes model + 0x268 (lea 0xA923FD, mov 0xA9240D).
            chooseListedPick(static_cast<std::byte*>(lock) - kModelLockOffset);
            return g_lockAcquire(lock, argument);
        }

        // Native answer (true) whenever nothing is listed.
        bool primaryRolloverHostsPanel(void*)
        {
            const auto flags = g_listedFlags.load(std::memory_order_relaxed);
            return !(flags & kListed) || (flags & kRightPicksListed);
        }

        // Native answer (false) whenever nothing is listed.
        bool secondaryRolloverHostsPanel(void*)
        {
            const auto flags = g_listedFlags.load(std::memory_order_relaxed);
            return (flags & kListed) && (flags & kLeftPicksListed);
        }

        std::uintptr_t rolloverHostSlotAddress(const RolloverHostSlot& host)
        {
            return REL::Offset(host.vtable).address() + sizeof(std::uintptr_t) * kHostsQuickContainerSlot;
        }

        bool rolloverHostSlotMatches(const RolloverHostSlot& host)
        {
            std::uintptr_t current{};
            std::array<std::uint8_t, 3> bytes{};
            const auto native = REL::Offset(host.native).address();
            return native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(rolloverHostSlotAddress(host)),
                       &current, sizeof(current)) &&
                current == native &&
                native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(native), bytes.data(),
                    bytes.size()) &&
                bytes == host.nativeBytes;
        }

        // Both prompts switch together or neither does.
        void installRolloverHosts()
        {
            if (!rolloverHostSlotMatches(kPrimaryRolloverHost) || !rolloverHostSlotMatches(kSecondaryRolloverHost)) {
                ROCK_LOG_WARN(Init,
                    "Quick loot panel: the rollover host slots do not match; "
                    "the left wand's Search prompt stays visible while the panel lists its container");
                return;
            }
            REL::safe_write(rolloverHostSlotAddress(kPrimaryRolloverHost),
                reinterpret_cast<std::uintptr_t>(&primaryRolloverHostsPanel));
            REL::safe_write(rolloverHostSlotAddress(kSecondaryRolloverHost),
                reinterpret_cast<std::uintptr_t>(&secondaryRolloverHostsPanel));
        }

        RE::NiPoint3 normalized(const RE::NiPoint3& v)
        {
            const float length = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
            return length > 0.0001f ? RE::NiPoint3{ v.x / length, v.y / length, v.z / length } : RE::NiPoint3{};
        }

        RE::NiPoint3 cross(const RE::NiPoint3& a, const RE::NiPoint3& b)
        {
            return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
        }

        /*
         * World pose of the panel's root at the listed container: on the
         * container's near side at its bound centre height, facing the eye.
         * The rows are the root's local axes in world space: X right, Y away
         * from the viewer, Z up, the frame PrimaryUIAttachNode gives it on the
         * hand.
         */
        std::optional<RE::NiTransform> panelAtContainer(const std::uint32_t handleValue, const RE::NiPoint3& eye,
            const RE::NiPoint3& eyeRight, const float handScale)
        {
            RE::ObjectRefHandle handle{};
            static_assert(sizeof(handle) == sizeof(handleValue));
            std::memcpy(&handle, &handleValue, sizeof(handle));
            const auto ref = handle.get();
            if (!ref) {
                return std::nullopt;
            }
            const auto* root = ref->Get3D();
            const bool bounded = root && root->worldBound.fRadius > 0.0f;
            const RE::NiPoint3 center = bounded ? root->worldBound.center : ref->data.location;
            const float radius = bounded ? root->worldBound.fRadius : 0.0f;

            RE::NiPoint3 toEye{ eye.x - center.x, eye.y - center.y, 0.0f };
            const float horizontal = std::sqrt(toEye.x * toEye.x + toEye.y * toEye.y);
            toEye = normalized(toEye);
            const float approach = (std::min)(radius, horizontal * kMaxApproachFraction);
            const RE::NiPoint3 position{ center.x + toEye.x * approach, center.y + toEye.y * approach, center.z };

            const RE::NiPoint3 view{ position.x - eye.x, position.y - eye.y, position.z - eye.z };
            const float distance = std::sqrt(view.x * view.x + view.y * view.y + view.z * view.z);
            if (distance < 1.0f) {
                return std::nullopt;
            }
            const RE::NiPoint3 forward = normalized(view);
            const RE::NiPoint3 kWorldUp{ 0.0f, 0.0f, 1.0f };
            RE::NiPoint3 right = cross(forward, kWorldUp);
            right = right.x * right.x + right.y * right.y + right.z * right.z > 0.0001f ? normalized(right) : eyeRight;
            const RE::NiPoint3 up = cross(right, forward);

            RE::NiTransform pose = transform_math::makeIdentityTransform<RE::NiTransform>();
            pose.rotate.entry[0][0] = right.x;
            pose.rotate.entry[0][1] = right.y;
            pose.rotate.entry[0][2] = right.z;
            pose.rotate.entry[1][0] = forward.x;
            pose.rotate.entry[1][1] = forward.y;
            pose.rotate.entry[1][2] = forward.z;
            pose.rotate.entry[2][0] = up.x;
            pose.rotate.entry[2][1] = up.y;
            pose.rotate.entry[2][2] = up.z;
            pose.translate = position;
            pose.scale = handScale * std::clamp(distance / kHandReadingDistance, 1.0f, kMaxViewScale);
            return pose;
        }

        void move(RE::NiAVObject* root, RE::NiNode* from, RE::NiNode* to)
        {
            RE::NiPointer<RE::NiAVObject> held;
            from->DetachChild(root, held);
            if (held) {
                to->AttachChild(held.get(), true);
            }
        }

        bool attachCarrier(RE::NiNode* room)
        {
            if (!g_carrier) {
                if (g_carrierFailed) {
                    return false;
                }
                g_carrier = native_scene::createEngineNiNode(1);
                if (!g_carrier) {
                    g_carrierFailed = true;
                    ROCK_LOG_ERROR(Hand,
                        "Quick loot: the container panel node could not be created; the panel stays on the right hand");
                    return false;
                }
            }
            if (g_carrier->parent != room) {
                if (auto* oldParent = g_carrier->parent) {
                    RE::NiPointer<RE::NiAVObject> detached;
                    oldParent->DetachChild(g_carrier.get(), detached);
                }
                room->AttachChild(g_carrier.get(), true);
            }
            return true;
        }

        void place(const f4vr::PlayerNodes& nodes)
        {
            auto* const model = *REL::Relocation<std::byte**>{ REL::Offset(kLootModel) };
            auto* const root = model ? *reinterpret_cast<RE::NiAVObject**>(model + kLootModelRootOffset) : nullptr;
            auto* const handAttach = nodes.primaryUIAttachNode;
            auto* const room = nodes.roomnode;
            auto* const eye = nodes.HmdNode;
            if (!root || !handAttach || !room || !eye) {
                return;
            }
            const bool onCarrier = g_carrier && root->parent == g_carrier.get();
            // Any other parent is a native rebuild or another owner; leave it.
            if (!onCarrier && root->parent != handAttach) {
                return;
            }

            const std::uint32_t listed = g_listedPick.load(std::memory_order_acquire);
            if (!listed) {
                // Nothing lists, so the panel is hidden; it stays where it was.
                return;
            }
            const RE::NiPoint3 eyeRight{ eye->world.rotate.entry[0][0], eye->world.rotate.entry[0][1],
                eye->world.rotate.entry[0][2] };
            const auto pose =
                panelAtContainer(listed, eye->world.translate, eyeRight, handAttach->world.scale * root->local.scale);
            if (!pose) {
                if (onCarrier) {
                    move(root, g_carrier.get(), handAttach);
                }
                ROCK_LOG_SAMPLE_DEBUG(Hand, 1000,
                    "Quick loot panel: the listed container has no reference; the panel is on the right hand");
                return;
            }
            if (!attachCarrier(room)) {
                return;
            }
            // carrier world = pose * root^-1; carrier local = room^-1 * carrier world.
            g_carrier->local = transform_math::composeTransforms(transform_math::invertTransform(room->world),
                transform_math::composeTransforms(*pose, transform_math::invertTransform(root->local)));
            if (!onCarrier) {
                move(root, handAttach, g_carrier.get());
                if (!g_onContainerLogged) {
                    g_onContainerLogged = true;
                    ROCK_LOG_INFO(Hand, "Quick loot panel moved from the hand to its container");
                }
            }
        }
    }

    bool install() noexcept
    {
        if (g_lockAcquire) {
            return true;
        }
        if (g_attempted) {
            return false;
        }
        g_attempted = true;

        for (const auto& expected : kExpectedPickCopy) {
            std::array<std::uint8_t, 7> live{};
            if (!native_memory::guardedCopyFromMemory(
                    reinterpret_cast<const void*>(REL::Offset(expected.address).address()), live.data(),
                    expected.size) ||
                std::memcmp(live.data(), expected.bytes.data(), expected.size) != 0) {
                ROCK_LOG_WARN(Init,
                    "Quick loot panel: the quick container pick copy at 0x{:X} does not match; "
                    "the panel stays on the right hand",
                    expected.address);
                return false;
            }
        }
        const auto callAddress = REL::Offset(kPicksCopiedLockCall).address();
        std::array<std::uint8_t, 5> call{};
        std::int32_t displacement{};
        if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(callAddress), call.data(),
                call.size()) ||
            call[0] != 0xE8) {
            ROCK_LOG_WARN(Init, "Quick loot panel: 0x{:X} is not a call; the panel stays on the right hand",
                kPicksCopiedLockCall);
            return false;
        }
        std::memcpy(&displacement, call.data() + 1, sizeof(displacement));
        if (callAddress + call.size() + displacement != REL::Offset(kSpinLockAcquire).address()) {
            ROCK_LOG_WARN(Init,
                "Quick loot panel: call 0x{:X} no longer reaches the model lock "
                "(another mod may own it); the panel stays on the right hand",
                kPicksCopiedLockCall);
            return false;
        }

        try {
            g_lockAcquire =
                reinterpret_cast<SpinLockAcquire>(F4SE::GetTrampoline().write_call<5>(callAddress, &afterPicksCopied));
        } catch (...) {
            REL::safe_write(callAddress, call.data(), call.size());
            g_lockAcquire = nullptr;
            ROCK_LOG_ERROR(Init, "Quick loot panel installation failed; the panel stays on the right hand");
            return false;
        }
        installRolloverHosts();
        ROCK_LOG_INFO(Init, "Quick loot panel installed: the loot panel stands at the container it lists");
        return true;
    }

    void update() noexcept
    {
        if (!g_lockAcquire) {
            return;
        }
        if (const auto* nodes = f4vr::getPlayerNodes()) {
            place(*nodes);
        }
    }
}
