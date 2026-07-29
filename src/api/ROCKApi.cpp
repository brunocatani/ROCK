#define ROCK_API_EXPORTS
#include "ROCKApi.h"

namespace rock::api
{
    ROCK_API const ROCKApi* ROCK_CALL ROCKAPI_GetApi()
    {
        return rock::provider::ROCKAPI_GetProviderApi();
    }
}
