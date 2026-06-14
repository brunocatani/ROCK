#pragma once

#include "ROCKProviderApi.h"

namespace rock::api
{
#define ROCK_API ROCK_PROVIDER_API
#define ROCK_CALL ROCK_PROVIDER_CALL

    inline constexpr std::uint32_t ROCK_API_VERSION = rock::provider::ROCK_PROVIDER_API_VERSION;
    using ROCKApi = rock::provider::RockProviderApi;

    ROCK_API const ROCKApi* ROCK_CALL ROCKAPI_GetApi();
}
