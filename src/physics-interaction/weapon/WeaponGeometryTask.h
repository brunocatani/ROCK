#pragma once

#include <coroutine>
#include <exception>
#include <utility>

namespace rock::weapon_geometry_work
{
    // Frame-thread-only cooperative work. The owning preparation destroys its
    // task before the referenced output/snapshots; cancellation unwinds every
    // suspended child through RAII. Exceptions are rethrown only to the owning
    // preparation's catch boundary, never into the game or physics callbacks.
    class Task
    {
    public:
        struct promise_type
        {
            std::exception_ptr failure;
            Task get_return_object() noexcept { return Task{ std::coroutine_handle<promise_type>::from_promise(*this) }; }
            std::suspend_always initial_suspend() const noexcept { return {}; }
            std::suspend_always final_suspend() const noexcept { return {}; }
            std::suspend_always yield_value(int) const noexcept { return {}; }
            void return_void() const noexcept {}
            void unhandled_exception() noexcept { failure = std::current_exception(); }
        };

        Task() = default;
        Task(Task&& other) noexcept : _handle(std::exchange(other._handle, {})) {}
        Task& operator=(Task&& other) noexcept
        {
            if (this != &other) {
                if (_handle) { _handle.destroy(); }
                _handle = std::exchange(other._handle, {});
            }
            return *this;
        }
        Task(const Task&) = delete;
        Task& operator=(const Task&) = delete;
        ~Task() { if (_handle) { _handle.destroy(); } }

        // true means suspended with more work, false means completed.
        bool step()
        {
            if (!_handle || _handle.done()) { return false; }
            _handle.resume();
            if (_handle.promise().failure) { std::rethrow_exception(_handle.promise().failure); }
            return !_handle.done();
        }

    private:
        explicit Task(std::coroutine_handle<promise_type> handle) noexcept : _handle(handle) {}
        std::coroutine_handle<promise_type> _handle{};
    };

    struct Quantum
    {
        unsigned work{};
        bool tick() noexcept { return (++work & 1023u) == 0; }
    };
}
