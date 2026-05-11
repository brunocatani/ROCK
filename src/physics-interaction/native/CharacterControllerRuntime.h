#pragma once

namespace RE
{
    class Actor;
    class bhkCharacterController;
}

namespace rock::character_controller_runtime
{
    RE::bhkCharacterController* tryGetActorCharacterController(RE::Actor* actor) noexcept;
    RE::bhkCharacterController* tryGetPlayerCharacterController() noexcept;
}
