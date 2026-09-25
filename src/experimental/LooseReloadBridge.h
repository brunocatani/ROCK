#pragma once
#include "experimental/LooseReloadProtocol.h"

namespace rock::loose_reload_bridge
{
    void publish(const loose_reload_experiment::Snapshot& value) noexcept;
    void clear() noexcept;
}
