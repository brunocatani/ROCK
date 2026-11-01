    api::Status captureInventoryWeapon(std::uint64_t, std::uint32_t form, std::uint32_t stack,
        api::weapon::InventoryWeapon& output) {
        const auto access = s_physicsInteraction.borrow();
        auto* pi = access.get();
        return pi && pi->isInitialized() ? pi->captureProviderInventoryWeapon(form, stack, output) : api::Status::NotReady;
    }
    api::Status requestInventoryEquip(std::uint64_t owner, const api::weapon::EquipRequest& request,
        std::uint64_t& command) {
        const auto access = s_physicsInteraction.borrow();
        auto* pi = access.get();
        return pi && pi->isInitialized() ? pi->requestProviderInventoryEquip(owner, request, command) : api::Status::NotReady;
    }
    api::Status getInventoryEquipResult(std::uint64_t owner, std::uint64_t command, api::weapon::EquipResult& output) {
        const auto access = s_physicsInteraction.borrow();
        auto* pi = access.get();
        return pi && pi->isInitialized() ? pi->getProviderInventoryEquipResult(owner, command, output) : api::Status::NotReady;
    }
    api::Status cancelInventoryEquip(std::uint64_t owner, std::uint64_t command) {
        const auto access = s_physicsInteraction.borrow();
        auto* pi = access.get();
        return pi && pi->isInitialized() ? pi->cancelProviderInventoryEquip(owner, command) : api::Status::NotReady;
    }
    api::Status unequipWeapon(std::uint32_t weaponSlot, std::uint32_t expectedFormId) {
        const auto access = s_physicsInteraction.borrow();
        auto* pi = access.get();
        return pi && pi->isInitialized() ? pi->unequipProviderWeapon(weaponSlot, expectedFormId) : api::Status::NotReady;
    }
    // The pose ROCK seats a firearm at in that hand (EquippedWeaponFrame,
    // FiringGrip): consumers presenting a weapon before ROCK reveals the
    // real one aim it here instead of guessing a grip.
    api::Status getFiringAimWorld(api::Hand hand, api::Transform& output) {
        const auto access = s_physicsInteraction.borrow();
        auto* pi = access.get();
        if (!pi || !pi->isInitialized()) return api::Status::NotReady;
        RE::NiTransform world{};
        const bool resolved = hand == api::Hand::Left ?
            TwoHandedGrip::tryGetLeftWeaponAimWorld(1.0f, world) :
            TwoHandedGrip::tryGetRightWeaponAimWorld(1.0f, world);
        if (!resolved) return api::Status::TargetUnavailable;
        for (int row = 0; row < 3; ++row)
            for (int column = 0; column < 3; ++column)
                output.rotate[static_cast<std::size_t>(row * 3 + column)] = world.rotate.entry[row][column];
        output.translate[0] = world.translate.x;
        output.translate[1] = world.translate.y;
        output.translate[2] = world.translate.z;
        output.scale = world.scale;
        return api::Status::Ok;
    }
