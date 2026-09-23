#include "physics-interaction/weapon/WeaponTextureAlphaCache.h"
#include "physics-interaction/PhysicsLog.h"

#include "RE/Bethesda/BSResourceNiBinaryStream.h"
#include "REL/Relocation.h"

#include <array>
#include <atomic>
#include <memory>
#include <thread>
#include <vector>
#include <limits>
#include <cstring>
#include <Windows.h>

namespace rock::weapon_texture_alpha
{
    namespace
    {
        using Result = texture_alpha::Result;
        constexpr std::size_t kMaximumBytes = 64 * 1024 * 1024;

        using NativeRead = std::size_t (*)(RE::BSResourceNiBinaryStream*, void*, std::size_t);
        std::size_t guardedRead(NativeRead read, RE::BSResourceNiBinaryStream* stream, void* buffer, std::size_t size)
        {
            __try {
                return read(stream, buffer, size);
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return (std::numeric_limits<std::size_t>::max)();
            }
        }

        std::size_t readResource(RE::BSResourceNiBinaryStream& stream, void* buffer, std::size_t size)
        {
            // VR vtable 142E57498 slot 5 -> 141C13110. Its raw code reads the
            // owned stream +10/buffer +18, advances +20 and returns bytes read.
            // The shared DoRead REL ID is absent from the VR address library.
            static REL::Relocation<NativeRead> nativeRead{ REL::Offset(0x1C13110) };
            return guardedRead(nativeRead.get(), &stream, buffer, size);
        }

        struct ResourceTrace
        {
            const char* stage = "open";
            std::size_t bytes = 0;
            std::uint32_t magic = 0;
            unsigned error = 0;
        };

        Result load(const char* path, const std::atomic<bool>& stopping, ResourceTrace& trace)
        {
            // Constructor 141C12B50 and destructor 141C12E10 own +10/+18;
            // native resource lookup preserves loose-file/archive precedence.
            RE::BSResourceNiBinaryStream stream(path);
            trace.error = static_cast<unsigned>(stream.lastError);
            if (!stream.good()) return Result::Unavailable;
            trace.stage = "read";
            std::vector<std::uint8_t> bytes;
            constexpr std::size_t chunk = 64 * 1024;
            while (bytes.size() < kMaximumBytes && !stopping.load(std::memory_order_acquire)) {
                const auto oldSize = bytes.size();
                bytes.resize(oldSize + chunk);
                const auto count = readResource(stream, bytes.data() + oldSize, chunk);
                trace.error = static_cast<unsigned>(stream.lastError);
                if (count > chunk) return Result::Unavailable;
                bytes.resize(oldSize + count);
                trace.bytes = bytes.size();
                if (count < chunk) {
                    trace.stage = "dds-alpha";
                    if (bytes.size() >= sizeof(trace.magic)) std::memcpy(&trace.magic, bytes.data(), sizeof(trace.magic));
                    return texture_alpha::inspect(bytes, &stopping);
                }
            }
            trace.stage = stopping.load(std::memory_order_acquire) ? "stopped" : "byte-limit";
            return Result::Unavailable;
        }

        class Cache
        {
        public:
            Cache() : _worker([this] { run(); }) {}
            ~Cache()
            {
                _stopping.store(true, std::memory_order_release);
                _wake.fetch_add(1, std::memory_order_release);
                _wake.notify_one();
                _worker.join();
            }

            Result query(std::string_view path)
            {
                // The game thread alone allocates stable slots. The worker
                // publishes just the final enum; lookups never take a lock.
                const auto count = _count.load(std::memory_order_relaxed);
                for (std::size_t i = 0; i < count; ++i) {
                    if (path == _entries[i].path.data())
                        return _entries[i].result.load(std::memory_order_acquire);
                }
                if (count == _entries.size()) {
                    if (!_capacityReported) {
                        _capacityReported = true;
                        ROCK_LOG_WARN(Weapon, "Weapon texture alpha cache full: entries={}; unknown textures remain visible", count);
                    }
                    return Result::Unavailable;
                }
                auto& entry = _entries[count];
                std::copy(path.begin(), path.end(), entry.path.begin());
                _count.store(count + 1, std::memory_order_release);
                _wake.fetch_add(1, std::memory_order_release);
                _wake.notify_one();
                return Result::Pending;
            }

        private:
            void run() noexcept
            {
                std::size_t next = 0;
                while (!_stopping.load(std::memory_order_acquire)) {
                    const auto sequence = _wake.load(std::memory_order_acquire);
                    if (next == _count.load(std::memory_order_acquire)) {
                        if (_stopping.load(std::memory_order_acquire)) break;
                        _wake.wait(sequence, std::memory_order_acquire);
                        continue;
                    }
                    auto& entry = _entries[next++];
                    auto result = Result::Unavailable;
                    ResourceTrace trace{};
                    try {
                        result = load(entry.path.data(), _stopping, trace);
                        ROCK_LOG_INFO(Weapon, "Weapon texture alpha: path='{}' result={} stage={} bytes={} magic={:08X} resourceError={}", entry.path.data(),
                            result == Result::Transparent ? "fully-transparent" : result == Result::Visible ? "has-visible-alpha" : "unavailable",
                            trace.stage, trace.bytes, trace.magic, trace.error);
                    } catch (const std::exception& error) {
                        ROCK_LOG_WARN(Weapon, "Weapon texture alpha failed: path='{}' reason='{}'", entry.path.data(), error.what());
                    } catch (...) {
                        ROCK_LOG_WARN(Weapon, "Weapon texture alpha failed: path='{}' reason=unknown", entry.path.data());
                    }
                    entry.result.store(result, std::memory_order_release);
                }
            }

            struct Entry
            {
                std::array<char, 260> path{};
                std::atomic<Result> result{ Result::Pending };
            };
            std::array<Entry, 1024> _entries{};
            std::atomic<std::size_t> _count{ 0 };
            std::atomic<unsigned> _wake{ 0 };
            std::atomic<bool> _stopping{ false };
            bool _capacityReported = false;
            std::thread _worker;
        };

        // Lifetime is explicit: WeaponCollision::init/shutdown, on the game
        // thread. A stop joins the worker before resource service ownership ends.
        std::unique_ptr<Cache> s_cache;
    }

    void start()
    {
        if (s_cache) return;
        try {
            s_cache = std::make_unique<Cache>();
        } catch (const std::exception& error) {
            ROCK_LOG_ERROR(Weapon, "Weapon texture alpha worker unavailable: {}", error.what());
        } catch (...) {
            ROCK_LOG_ERROR(Weapon, "Weapon texture alpha worker unavailable: unknown exception");
        }
    }

    void stop() { s_cache.reset(); }

    Result query(std::string_view sourcePath)
    {
        if (!s_cache || sourcePath.empty()) return Result::Unavailable;
        std::array<char, 260> path{};
        std::string_view prefix = "textures/";
        const auto normalized = [](char c) {
            if (c == '\\') return '/';
            return c >= 'A' && c <= 'Z' ? static_cast<char>(c + ('a' - 'A')) : c;
        };
        bool hasPrefix = sourcePath.size() >= prefix.size();
        for (std::size_t i = 0; hasPrefix && i < prefix.size(); ++i)
            hasPrefix &= normalized(sourcePath[i]) == prefix[i];
        if (hasPrefix) sourcePath.remove_prefix(prefix.size());
        if (prefix.size() + sourcePath.size() >= path.size()) return Result::Unavailable;
        std::copy(prefix.begin(), prefix.end(), path.begin());
        for (std::size_t i = 0; i < sourcePath.size(); ++i) path[prefix.size() + i] = normalized(sourcePath[i]);
        return s_cache->query({ path.data(), prefix.size() + sourcePath.size() });
    }
}
