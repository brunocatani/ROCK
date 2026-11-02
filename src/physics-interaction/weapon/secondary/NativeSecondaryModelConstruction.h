#pragma once

namespace rock::native_secondary_model
{
    // Installs scoped adapters in the existing inventory weapon builder.
    // Native code keeps all model, biped, resource and graph ownership.
    [[nodiscard]] bool install() noexcept;
}
