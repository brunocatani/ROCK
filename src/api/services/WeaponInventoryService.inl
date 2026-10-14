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
