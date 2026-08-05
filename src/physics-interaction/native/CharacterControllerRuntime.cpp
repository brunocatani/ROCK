#include "physics-interaction/native/CharacterControllerRuntime.h"

#include "physics-interaction/native/PhysicsScale.h"

#include "RE/Bethesda/PlayerCharacter.h"

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <windows.h>

namespace rock::character_controller_runtime
{
    namespace
    {
        // FO4VR 1.2.72 layout.  CommonLibF4VR still declares the flat-game
        // MiddleHighProcessData::charController offset (0x3E0), so accessing
        // that member returns the adjacent VR field and cannot identify the
        // controller passed to bhkCharProxyController's contact callback.
        constexpr std::ptrdiff_t kActorCurrentProcessOffset = 0x300;
        constexpr std::ptrdiff_t kProcessMiddleHighOffset = 0x08;
        constexpr std::ptrdiff_t kMiddleHighCharacterControllerOffset = 0x3E8;

        RE::bhkCharacterController* tryGetActorCharacterControllerRaw(RE::Actor* actor) noexcept
        {
            if (!actor) {
                return nullptr;
            }

            const auto* actorBytes = reinterpret_cast<const std::uint8_t*>(actor);
            const void* currentProcess =
                *reinterpret_cast<void* const*>(actorBytes + kActorCurrentProcessOffset);
            if (!currentProcess) {
                return nullptr;
            }

            const auto* processBytes = reinterpret_cast<const std::uint8_t*>(currentProcess);
            const void* middleHigh =
                *reinterpret_cast<void* const*>(processBytes + kProcessMiddleHighOffset);
            if (!middleHigh) {
                return nullptr;
            }

            const auto* middleHighBytes = reinterpret_cast<const std::uint8_t*>(middleHigh);
            return *reinterpret_cast<RE::bhkCharacterController* const*>(
                middleHighBytes + kMiddleHighCharacterControllerOffset);
        }
    }

    RE::bhkCharacterController* tryGetActorCharacterController(RE::Actor* actor) noexcept
    {
        RE::bhkCharacterController* controller = nullptr;

        __try {
            controller = tryGetActorCharacterControllerRaw(actor);
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            controller = nullptr;
        }

        return controller;
    }

    RE::bhkCharacterController* tryGetPlayerCharacterController() noexcept
    {
        return tryGetActorCharacterController(RE::PlayerCharacter::GetSingleton());
    }

    bool tryGetPlayerLocomotionVelocityRawGameUnits(RE::NiPoint3& outVelocityGameUnits) noexcept
    {
        outVelocityGameUnits = RE::NiPoint3{};
        bool ok = false;

        __try {
            auto* player = RE::PlayerCharacter::GetSingleton();
            auto* charController = tryGetActorCharacterControllerRaw(player);
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
            auto* charController = tryGetActorCharacterControllerRaw(player);
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
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            ok = false;
        }

        return ok;
    }
}
