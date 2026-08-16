#include "physics-interaction/native/CharacterControllerRuntime.h"

#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/bhkCharacterController.h"

#include <cmath>

#include <windows.h>

namespace rock::character_controller_runtime
{
    namespace
    {
        constexpr float kMaxPlausibleControllerCoordinateHavok = 1'000'000.0f;

        bool isPlausibleControllerPosition(const RE::hkVector4f& position) noexcept
        {
            return std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z) &&
                   std::fabs(position.x) <= kMaxPlausibleControllerCoordinateHavok &&
                   std::fabs(position.y) <= kMaxPlausibleControllerCoordinateHavok &&
                   std::fabs(position.z) <= kMaxPlausibleControllerCoordinateHavok;
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
        return tryGetActorCharacterController(RE::PlayerCharacter::GetSingleton());
    }

    CharacterControllerPositionSample samplePlayerCharacterControllerPositionHavok() noexcept
    {
        CharacterControllerPositionSample sample{};

        __try {
            auto* controller = tryGetPlayerCharacterController();
            if (!controller) {
                return sample;
            }

            const auto identity = reinterpret_cast<std::uintptr_t>(controller);
            const auto vtable = *reinterpret_cast<const std::uintptr_t*>(controller);
            if (identity == 0 || vtable == 0) {
                return sample;
            }

            RE::hkVector4f positionHavok{};
            controller->GetPositionImpl(positionHavok, false);
            if (!isPlausibleControllerPosition(positionHavok)) {
                return sample;
            }

            sample.positionHavok = RE::NiPoint3{ positionHavok.x, positionHavok.y, positionHavok.z };
            sample.controllerIdentity = identity;
            sample.controllerVtable = vtable;
            sample.valid = true;
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            sample = {};
        }

        return sample;
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

}
