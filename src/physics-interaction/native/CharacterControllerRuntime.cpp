#include "physics-interaction/native/CharacterControllerRuntime.h"

#include "RE/Bethesda/PlayerCharacter.h"

#include <cmath>

#include <windows.h>

namespace rock::character_controller_runtime
{
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
