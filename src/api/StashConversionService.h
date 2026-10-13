#pragma once
#include <ROCK/Storage.h>
namespace rock::stash_conversion {
// Storage registerStashConverter.
api::Status ROCK_CALL registerConverter(api::OwnerToken owner,api::storage::StashConverter converter,void* userData) noexcept;
api::storage::StashResult convert(std::uint32_t reference,std::uint32_t base) noexcept;
}
