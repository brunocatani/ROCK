#pragma once

#include "api/FRIKApi.h"

namespace rock
{
    inline frik::api::FRIKApi::Hand handFromBool(bool isLeft) { return isLeft ? frik::api::FRIKApi::Hand::Left : frik::api::FRIKApi::Hand::Right; }
}
