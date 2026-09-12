#include "physics-interaction/native/CharacterControllerRuntime.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/PhysicsScale.h"

#include "RE/Bethesda/PlayerCharacter.h"

#include <REL/Relocation.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>

#include <windows.h>

namespace rock::character_controller_runtime
{
    namespace
    {
        constexpr std::uintptr_t kProxyControllerVtableRva = 0x2E89328;
        constexpr std::uintptr_t kRigidControllerVtableRva = 0x2E89B28;
        constexpr std::uintptr_t kProxyPenetrationFunctionRva = 0x1E4E970;
        constexpr std::uintptr_t kRigidPenetrationFunctionRva = 0x1E54390;
        constexpr std::uintptr_t kPlayerJumpFunctionRva = 0x1E216E0;
        constexpr std::size_t kPenetrationVtableSlotOffset = 0x1E8;
        constexpr std::size_t kCharacterImplementationOffset = 0x470;
        constexpr std::size_t kCharacterImplementationPositionOffset = 0x70;
        constexpr std::size_t kCachedVelocityOffset = 0x250;
        constexpr std::size_t kSurfaceSupportedStateOffset = 0x271;
        constexpr std::size_t kSurfaceNormalOffset = 0x280;
        constexpr std::size_t kRadiusOffset = 0x370;
        constexpr std::size_t kHeightOffset = 0x374;
        constexpr float kMaximumCoordinateGame = 1.0e7f;

        enum class ControllerResolveStage : std::uint8_t
        {
            None,
            Player,
            CurrentProcess,
            MiddleHigh,
            Controller,
            Dispatch,
            CharacterImplementation,
            Complete,
        };

        std::atomic<ControllerResolveStage> s_lastControllerResolveStage{
            ControllerResolveStage::Complete
        };

        enum class ControllerDispatchFailure : std::uint8_t
        {
            None,
            UnsupportedRuntime,
            UnknownVtable,
            SlotMismatch,
            PrefixMismatch,
            ReadFault,
        };

        struct ControllerDispatchDiagnostic
        {
            ControllerDispatchFailure failure{
                ControllerDispatchFailure::None
            };
            std::uintptr_t moduleBase{ 0 };
            std::uintptr_t actualVtable{ 0 };
            std::uintptr_t expectedVtable{ 0 };
            std::uintptr_t actualFunction{ 0 };
            std::uintptr_t expectedFunction{ 0 };
        };

        std::atomic<std::uint64_t> s_lastDispatchFailureSignature{ 0 };

        [[nodiscard]] const char* resolveStageName(
            const ControllerResolveStage stage) noexcept
        {
            switch (stage) {
            case ControllerResolveStage::None:
                return "none";
            case ControllerResolveStage::Player:
                return "player";
            case ControllerResolveStage::CurrentProcess:
                return "currentProcess";
            case ControllerResolveStage::MiddleHigh:
                return "middleHigh";
            case ControllerResolveStage::Controller:
                return "controller";
            case ControllerResolveStage::Dispatch:
                return "controllerDispatch";
            case ControllerResolveStage::CharacterImplementation:
                return "characterImplementation";
            case ControllerResolveStage::Complete:
                return "complete";
            }
            return "unknown";
        }

        void observeResolveStage(
            const ControllerResolveStage stage) noexcept
        {
            const auto previous = s_lastControllerResolveStage.exchange(
                stage,
                std::memory_order_acq_rel);
            if (stage == previous) {
                return;
            }
            if (stage == ControllerResolveStage::Complete) {
                ROCK_LOG_INFO(PlayerController,
                    "FO4VR player-controller chain recovered after deepest stage '{}'.",
                    resolveStageName(previous));
                return;
            }
            ROCK_LOG_WARN(PlayerController,
                "FO4VR player-controller chain failed closed at deepest verified stage '{}'.",
                resolveStageName(stage));
        }

        [[nodiscard]] const char* dispatchFailureName(
            const ControllerDispatchFailure failure) noexcept
        {
            switch (failure) {
            case ControllerDispatchFailure::None:
                return "none";
            case ControllerDispatchFailure::UnsupportedRuntime:
                return "unsupportedRuntime";
            case ControllerDispatchFailure::UnknownVtable:
                return "unknownVtable";
            case ControllerDispatchFailure::SlotMismatch:
                return "slotMismatch";
            case ControllerDispatchFailure::PrefixMismatch:
                return "prefixMismatch";
            case ControllerDispatchFailure::ReadFault:
                return "readFault";
            }
            return "unknown";
        }

        void observeDispatchFailure(
            const RE::bhkCharacterController* controller,
            const ControllerDispatchDiagnostic& diagnostic) noexcept
        {
            std::uint64_t signature =
                static_cast<std::uint64_t>(diagnostic.failure);
            signature ^= diagnostic.actualVtable;
            signature ^= diagnostic.actualFunction << 1;
            if (signature == 0) {
                signature = 1;
            }
            if (s_lastDispatchFailureSignature.exchange(
                    signature,
                    std::memory_order_acq_rel) == signature) {
                return;
            }
            ROCK_LOG_WARN(PlayerController,
                "Controller dispatch rejected: reason={} controller=0x{:X} moduleBase=0x{:X} vtable=0x{:X} expectedVtable=0x{:X} slot=0x{:X} function=0x{:X} expectedFunction=0x{:X}.",
                dispatchFailureName(diagnostic.failure),
                reinterpret_cast<std::uintptr_t>(controller),
                diagnostic.moduleBase,
                diagnostic.actualVtable,
                diagnostic.expectedVtable,
                kPenetrationVtableSlotOffset,
                diagnostic.actualFunction,
                diagnostic.expectedFunction);
        }

        [[nodiscard]] bool plausiblePointerWitness(
            const void* pointer) noexcept
        {
            const auto address = reinterpret_cast<std::uintptr_t>(pointer);
            return address >= 0x1'0000 && (address & 0x7u) == 0;
        }

        struct ControllerDispatchSpec
        {
            std::uintptr_t vtableRva{};
            std::uintptr_t penetrationFunctionRva{};
            PlayerControllerImplementation implementation{
                PlayerControllerImplementation::Unknown
            };
            std::array<std::uint8_t, 11> penetrationPrefix{};
        };

        // Fallout4VR.exe.unpacked.exe
        // SHA-256 95ABB321DD5F8E9536665D8EA0A8DC550B8A5C7878516F3A1727569844B0BBF6.
        // The proxy entry starts with a redundant REX prefix (0x40) that is
        // omitted by textual disassembly but is present in the live image.
        constexpr std::array<ControllerDispatchSpec, 2> kControllerDispatchSpecs{
            ControllerDispatchSpec{
                kProxyControllerVtableRva,
                kProxyPenetrationFunctionRva,
                PlayerControllerImplementation::Proxy,
                { 0x40, 0x53, 0x41, 0x56, 0x48, 0x81, 0xEC, 0x68, 0x04, 0x00, 0x00 },
            },
            ControllerDispatchSpec{
                kRigidControllerVtableRva,
                kRigidPenetrationFunctionRva,
                PlayerControllerImplementation::RigidBody,
                { 0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x10, 0x57 },
            },
        };

        [[nodiscard]] bool finitePoint(const RE::NiPoint3& point) noexcept
        {
            return std::isfinite(point.x) && std::isfinite(point.y) &&
                   std::isfinite(point.z);
        }

        [[nodiscard]] bool validCoordinate(
            const RE::NiPoint3& point) noexcept
        {
            return finitePoint(point) &&
                   std::abs(point.x) < kMaximumCoordinateGame &&
                   std::abs(point.y) < kMaximumCoordinateGame &&
                   std::abs(point.z) < kMaximumCoordinateGame;
        }

        [[nodiscard]] RE::bhkCharacterController*
            tryResolvePlayerControllerRaw(
                ControllerResolveStage* outStage = nullptr) noexcept
        {
            RE::bhkCharacterController* controller = nullptr;
            ControllerResolveStage stage = ControllerResolveStage::None;
            __try {
                auto* player = RE::PlayerCharacter::GetSingleton();
                if (plausiblePointerWitness(player)) {
                    stage = ControllerResolveStage::Player;
                    const auto* playerBytes =
                        reinterpret_cast<const std::uint8_t*>(player);
                    const void* currentProcess =
                        *reinterpret_cast<void* const*>(playerBytes + 0x300);
                    if (plausiblePointerWitness(currentProcess)) {
                        stage = ControllerResolveStage::CurrentProcess;
                        const auto* processBytes =
                            reinterpret_cast<const std::uint8_t*>(
                                currentProcess);
                        const void* middleHigh =
                            *reinterpret_cast<void* const*>(
                                processBytes + 0x08);
                        if (plausiblePointerWitness(middleHigh)) {
                            stage = ControllerResolveStage::MiddleHigh;
                            const auto* middleHighBytes =
                                reinterpret_cast<const std::uint8_t*>(
                                    middleHigh);
                            auto* resolved = *reinterpret_cast<
                                RE::bhkCharacterController* const*>(
                                    middleHighBytes + 0x3E8);
                            if (plausiblePointerWitness(resolved)) {
                                controller = resolved;
                                stage = ControllerResolveStage::Controller;
                            }
                        }
                    }
                }
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                controller = nullptr;
            }
            if (outStage) {
                *outStage = stage;
            }
            return controller;
        }

        [[nodiscard]] const ControllerDispatchSpec* resolveDispatch(
            RE::bhkCharacterController* controller,
            std::uintptr_t& outPenetrationFunction,
            ControllerDispatchDiagnostic* outDiagnostic = nullptr) noexcept
        {
            outPenetrationFunction = 0;
            ControllerDispatchDiagnostic diagnostic{};
            if (!controller || !REL::Module::IsVR() ||
                REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) {
                diagnostic.failure =
                    ControllerDispatchFailure::UnsupportedRuntime;
                if (outDiagnostic) {
                    *outDiagnostic = diagnostic;
                }
                return nullptr;
            }

            const auto moduleBase = REL::Module::get().base();
            diagnostic.moduleBase = moduleBase;
            const ControllerDispatchSpec* matched = nullptr;
            __try {
                const auto vtable = *reinterpret_cast<const std::uintptr_t*>(
                    controller);
                diagnostic.actualVtable = vtable;
                for (const auto& spec : kControllerDispatchSpecs) {
                    if (vtable != moduleBase + spec.vtableRva) {
                        continue;
                    }
                    diagnostic.expectedVtable =
                        moduleBase + spec.vtableRva;
                    const auto function =
                        *reinterpret_cast<const std::uintptr_t*>(
                            vtable + kPenetrationVtableSlotOffset);
                    diagnostic.actualFunction = function;
                    diagnostic.expectedFunction =
                        moduleBase + spec.penetrationFunctionRva;
                    if (function != diagnostic.expectedFunction) {
                        diagnostic.failure =
                            ControllerDispatchFailure::SlotMismatch;
                        break;
                    }
                    if (std::memcmp(
                            reinterpret_cast<const void*>(function),
                            spec.penetrationPrefix.data(),
                            spec.penetrationPrefix.size()) != 0) {
                        diagnostic.failure =
                            ControllerDispatchFailure::PrefixMismatch;
                        break;
                    }
                    matched = &spec;
                    outPenetrationFunction = function;
                    diagnostic.failure = ControllerDispatchFailure::None;
                    break;
                }
                if (!matched &&
                    diagnostic.failure == ControllerDispatchFailure::None) {
                    diagnostic.failure =
                        ControllerDispatchFailure::UnknownVtable;
                }
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                matched = nullptr;
                outPenetrationFunction = 0;
                diagnostic.failure = ControllerDispatchFailure::ReadFault;
            }
            if (outDiagnostic) {
                *outDiagnostic = diagnostic;
            }
            return matched;
        }

        [[nodiscard]] bool validateJumpFunction(
            std::uintptr_t& outFunction) noexcept
        {
            outFunction = 0;
            if (!REL::Module::IsVR() ||
                REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) {
                return false;
            }
            const auto function =
                REL::Module::get().base() + kPlayerJumpFunctionRva;
            bool valid = false;
            __try {
                const auto* bytes =
                    reinterpret_cast<const std::uint8_t*>(function);
                valid =
                    bytes[0] == 0xF3 && bytes[1] == 0x0F &&
                    bytes[2] == 0x10 && bytes[3] == 0x05 &&
                    bytes[8] == 0xC7 && bytes[9] == 0x81 &&
                    bytes[10] == 0x04 && bytes[11] == 0x03 &&
                    bytes[12] == 0x00 && bytes[13] == 0x00 &&
                    bytes[14] == 0x01 && bytes[15] == 0x00 &&
                    bytes[16] == 0x00 && bytes[17] == 0x00 &&
                    bytes[18] == 0xF3 && bytes[19] == 0x0F &&
                    bytes[20] == 0x59 && bytes[21] == 0xC1 &&
                    bytes[22] == 0xF3 && bytes[23] == 0x0F &&
                    bytes[24] == 0x11 && bytes[25] == 0x81 &&
                    bytes[26] == 0x24 && bytes[27] == 0x03 &&
                    bytes[28] == 0x00 && bytes[29] == 0x00 &&
                    bytes[30] == 0xC3;
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                valid = false;
            }
            if (valid) {
                outFunction = function;
            }
            return valid;
        }
    }

    RE::bhkCharacterController* tryGetActorCharacterController(RE::Actor* actor) noexcept
    {
        RE::bhkCharacterController* controller = nullptr;

        __try {
            if (actor && actor->currentProcess && actor->currentProcess->middleHigh) {
                controller = actor->currentProcess->middleHigh->charController.get();
            }
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            controller = nullptr;
        }

        return controller;
    }

    RE::bhkCharacterController* tryGetPlayerCharacterController() noexcept
    {
        return tryResolvePlayerControllerRaw();
    }

    bool tryGetPlayerControllerState(PlayerControllerState& outState) noexcept
    {
        outState = {};
        const float havokToGame = physics_scale::havokToGame();
        if (!std::isfinite(havokToGame) || havokToGame <= 0.0f) {
            return false;
        }

        ControllerResolveStage resolveStage = ControllerResolveStage::None;
        auto* controller = tryResolvePlayerControllerRaw(&resolveStage);
        if (!controller) {
            observeResolveStage(resolveStage);
            return false;
        }
        std::uintptr_t penetrationFunction = 0;
        ControllerDispatchDiagnostic dispatchDiagnostic{};
        const auto* dispatch = resolveDispatch(
            controller,
            penetrationFunction,
            &dispatchDiagnostic);
        if (!dispatch) {
            observeDispatchFailure(controller, dispatchDiagnostic);
            observeResolveStage(ControllerResolveStage::Dispatch);
            return false;
        }
        s_lastDispatchFailureSignature.store(0, std::memory_order_release);

        bool read = false;
        __try {
            const auto* bytes = reinterpret_cast<const std::uint8_t*>(
                controller);
            const void* implementation =
                *reinterpret_cast<void* const*>(
                    bytes + kCharacterImplementationOffset);
            if (!plausiblePointerWitness(implementation)) {
                observeResolveStage(
                    ControllerResolveStage::CharacterImplementation);
                return false;
            }
            const auto* implementationBytes =
                reinterpret_cast<const std::uint8_t*>(implementation);
            const auto positionHavok = *reinterpret_cast<const RE::NiPoint3*>(
                implementationBytes +
                kCharacterImplementationPositionOffset);
            outState.positionGame = RE::NiPoint3{
                positionHavok.x * havokToGame,
                positionHavok.y * havokToGame,
                positionHavok.z * havokToGame,
            };
            outState.positionValid = validCoordinate(outState.positionGame);

            outState.velocityGame = *reinterpret_cast<const RE::NiPoint3*>(
                bytes + kCachedVelocityOffset);
            outState.velocityValid = finitePoint(outState.velocityGame);

            const auto supportValue =
                *(bytes + kSurfaceSupportedStateOffset);
            if (supportValue <= static_cast<std::uint8_t>(
                    PlayerSupportState::Supported)) {
                outState.supportState =
                    static_cast<PlayerSupportState>(supportValue);
            }
            outState.supportNormal = *reinterpret_cast<const RE::NiPoint3*>(
                bytes + kSurfaceNormalOffset);
            const float supportLengthSquared =
                outState.supportNormal.x * outState.supportNormal.x +
                outState.supportNormal.y * outState.supportNormal.y +
                outState.supportNormal.z * outState.supportNormal.z;
            outState.supportNormalValid =
                finitePoint(outState.supportNormal) &&
                std::isfinite(supportLengthSquared) &&
                supportLengthSquared >= 0.25f &&
                supportLengthSquared <= 2.25f;

            const float radiusHavok =
                *reinterpret_cast<const float*>(bytes + kRadiusOffset);
            const float heightHavok =
                *reinterpret_cast<const float*>(bytes + kHeightOffset);
            outState.radiusGame = radiusHavok * havokToGame;
            outState.heightGame = heightHavok * havokToGame;
            outState.shapeValid =
                std::isfinite(outState.radiusGame) &&
                std::isfinite(outState.heightGame) &&
                outState.radiusGame > 0.0f &&
                outState.radiusGame <= 256.0f &&
                outState.heightGame > 0.0f &&
                outState.heightGame <= 512.0f;
            outState.controllerIdentity =
                reinterpret_cast<std::uintptr_t>(controller);
            outState.implementation = dispatch->implementation;
            read = outState.positionValid && outState.velocityValid;
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            read = false;
        }
        if (!read) {
            outState = {};
            return false;
        }

        outState.valid = true;
        observeResolveStage(ControllerResolveStage::Complete);
        return true;
    }

    bool requestPlayerJump(const float heightGameUnits) noexcept
    {
        if (!std::isfinite(heightGameUnits) ||
            heightGameUnits <= 0.0f || heightGameUnits > 256.0f) {
            return false;
        }
        auto* controller = tryResolvePlayerControllerRaw();
        std::uintptr_t penetrationFunction = 0;
        if (!resolveDispatch(controller, penetrationFunction)) {
            return false;
        }
        std::uintptr_t jumpFunction = 0;
        if (!validateJumpFunction(jumpFunction)) {
            return false;
        }

        bool requested = false;
        __try {
            using JumpFunction = void (*)(
                RE::bhkCharacterController*,
                float);
            reinterpret_cast<JumpFunction>(jumpFunction)(
                controller,
                heightGameUnits);
            requested = true;
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            requested = false;
        }
        return requested;
    }

    bool tryGetPlayerLocomotionVelocityRawGameUnits(RE::NiPoint3& outVelocityGameUnits) noexcept
    {
        outVelocityGameUnits = RE::NiPoint3{};
        bool ok = false;

        __try {
            auto* player = RE::PlayerCharacter::GetSingleton();
            if (player) {
                // Verified FO4VR offsets only -- do not substitute CommonLibF4VR struct members here.
                const auto* playerBytes = reinterpret_cast<const std::uint8_t*>(player);
                const void* currentProcess = *reinterpret_cast<void* const*>(playerBytes + 0x300);  // Actor::currentProcess
                if (currentProcess) {
                    const auto* processBytes = reinterpret_cast<const std::uint8_t*>(currentProcess);
                    const void* middleHigh = *reinterpret_cast<void* const*>(processBytes + 0x08);  // AIProcess::middleHigh
                    if (middleHigh) {
                        const auto* middleHighBytes = reinterpret_cast<const std::uint8_t*>(middleHigh);
                        const void* charController = *reinterpret_cast<void* const*>(middleHighBytes + 0x3E8);  // VR-verified (CommonLib 0x3E0 is wrong)
                        if (charController) {
                            const auto* ccBytes = reinterpret_cast<const std::uint8_t*>(charController);
                            const float vx = *reinterpret_cast<const float*>(ccBytes + 0x250);  // cachedLinearVelocity.x (game units)
                            const float vy = *reinterpret_cast<const float*>(ccBytes + 0x254);
                            const float vz = *reinterpret_cast<const float*>(ccBytes + 0x258);
                            if (std::isfinite(vx) && std::isfinite(vy) && std::isfinite(vz)) {
                                outVelocityGameUnits.x = vx;
                                outVelocityGameUnits.y = vy;
                                outVelocityGameUnits.z = vz;
                                ok = true;
                            }
                        }
                    }
                }
            }
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            ok = false;
        }

        return ok;
    }

    bool tryGetPlayerActorPositionGameUnits(RE::NiPoint3& outPositionGameUnits) noexcept
    {
        outPositionGameUnits = RE::NiPoint3{};
        bool ok = false;

        __try {
            auto* player = RE::PlayerCharacter::GetSingleton();
            if (player) {
                const RE::NiPoint3 position = player->GetPosition();
                if (std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z)) {
                    outPositionGameUnits = position;
                    ok = true;
                }
            }
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            ok = false;
        }

        return ok;
    }

    bool tryGetPlayerRoomAnchorPositionGameUnits(RE::NiPoint3& outPositionGameUnits) noexcept
    {
        outPositionGameUnits = RE::NiPoint3{};
        bool ok = false;

        const float havokToGame = physics_scale::havokToGame();
        if (!std::isfinite(havokToGame) || havokToGame <= 0.0f) {
            return false;
        }

        __try {
            auto* player = RE::PlayerCharacter::GetSingleton();
            if (player) {
                // Same VR-verified walk as the velocity read above.
                const auto* playerBytes = reinterpret_cast<const std::uint8_t*>(player);
                const void* currentProcess = *reinterpret_cast<void* const*>(playerBytes + 0x300);
                if (currentProcess) {
                    const auto* processBytes = reinterpret_cast<const std::uint8_t*>(currentProcess);
                    const void* middleHigh = *reinterpret_cast<void* const*>(processBytes + 0x08);
                    if (middleHigh) {
                        const auto* middleHighBytes = reinterpret_cast<const std::uint8_t*>(middleHigh);
                        const void* charController = *reinterpret_cast<void* const*>(middleHighBytes + 0x3E8);
                        if (charController) {
                            const auto* ccBytes = reinterpret_cast<const std::uint8_t*>(charController);
                            // GetPositionImpl's own indirection: character impl, then its position.
                            const void* characterImpl = *reinterpret_cast<void* const*>(ccBytes + 0x470);
                            if (characterImpl) {
                                const auto* implBytes = reinterpret_cast<const std::uint8_t*>(characterImpl);
                                const float x = *reinterpret_cast<const float*>(implBytes + 0x70);
                                const float y = *reinterpret_cast<const float*>(implBytes + 0x74);
                                const float z = *reinterpret_cast<const float*>(implBytes + 0x78);
                                if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                                    outPositionGameUnits.x = x * havokToGame;
                                    outPositionGameUnits.y = y * havokToGame;
                                    outPositionGameUnits.z = z * havokToGame;
                                    ok = true;
                                }
                            }
                        }
                    }
                }
            }
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            ok = false;
        }

        return ok;
    }
}
