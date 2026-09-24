Status ROCK_CALL captureInventoryWeapon(OwnerToken owner, std::uint32_t form, std::uint32_t stack,
    v1_1::InventoryWeapon* output) noexcept {
    if (const auto status = checkOutput(output); status != Status::Ok) return status;
    return invoke(owner, kInterfaceId, 1, true, [&]() {
        return provider::runtime::captureInventoryWeapon(owner, form, stack, *output);
    }, provider::OwnerAccess::Active);
}
Status ROCK_CALL requestInventoryEquip(OwnerToken owner, const v1_1::EquipRequest* request,
    std::uint64_t* command) noexcept {
    if (!command) return Status::InvalidArgument;
    *command = 0;
    if (const auto status = checkInput(request); status != Status::Ok) return status;
    return invoke(owner, kInterfaceId, 2, true, [&]() {
        return provider::runtime::requestInventoryEquip(owner, *request, *command);
    }, provider::OwnerAccess::Active);
}
Status ROCK_CALL getInventoryEquipResult(OwnerToken owner, std::uint64_t command, v1_1::EquipResult* output) noexcept {
    if (const auto status = checkOutput(output); status != Status::Ok) return status;
    return invoke(owner, kInterfaceId, 1, true, [&]() {
        return provider::runtime::getInventoryEquipResult(owner, command, *output);
    });
}
Status ROCK_CALL cancelInventoryEquip(OwnerToken owner, std::uint64_t command) noexcept {
    return invoke(owner, kInterfaceId, 2, true, [&]() {
        return provider::runtime::cancelInventoryEquip(owner, command);
    });
}
