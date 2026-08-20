#pragma once

#include <array>
#include <filesystem>
#include <memory>

namespace rock::monitor
{
    struct MonitorSettings
    {
        // Compiled fallback stays rotation-neutral so it is always consistent
        // with the identity localOrientation below; the full tuned pose
        // (rotY=90, rotZ=6, flips X/Y) ships in the default INI template and
        // is derived properly on load.
        bool enabled{ false };
        float positionX{ 7.75f };
        float positionY{ 9.0f };
        float positionZ{ -16.5f };
        float rotationXDegrees{ 0.0f };
        float rotationYDegrees{ 0.0f };
        float rotationZDegrees{ 0.0f };
        bool flipRotationX{ false };
        bool flipRotationY{ false };
        bool flipRotationZ{ false };
        // Derived once on the config worker from configured qZ*qY*qX followed by
        // explicit final-panel-local X/Y/Z flip postmultiplication. The producer
        // frame hook consumes this normalized [x,y,z,w] quaternion without
        // trigonometry.
        std::array<float, 4> localOrientation{ 0.0f, 0.0f, 0.0f, 1.0f };
    };

    // Owns the Monitor INI and its file watcher. File parsing happens only during
    // startup or on FileWatch's worker thread; the producer frame hook only loads
    // the immutable published snapshot.
    class MonitorConfig final
    {
    public:
        MonitorConfig();
        ~MonitorConfig();

        MonitorConfig(const MonitorConfig&) = delete;
        MonitorConfig& operator=(const MonitorConfig&) = delete;

        [[nodiscard]] bool load();
        void stop() noexcept;

        [[nodiscard]] std::shared_ptr<const MonitorSettings> snapshot() const noexcept;
        [[nodiscard]] const std::filesystem::path& path() const noexcept;

    private:
        struct Impl;
        std::unique_ptr<Impl> _impl;
    };
}
