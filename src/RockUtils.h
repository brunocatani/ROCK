#pragma once

#include "api/FRIKApi.h"

namespace frik::rock
{
    inline frik::api::FRIKApi::Hand handFromBool(bool isLeft)
    {
        return isLeft ? frik::api::FRIKApi::Hand::Left : frik::api::FRIKApi::Hand::Right;
    }
}
