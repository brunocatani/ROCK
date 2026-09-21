#pragma once

namespace rock::provider::borrowed_callback {
    inline thread_local bool active=false;
    // Synchronous borrowed-data visitors must not re-enter ROCK or mutate the
    // registrations/storage whose data they are currently observing.
    class Scope {
        bool _previous{active};
    public:
        Scope() noexcept { active=true; }
        ~Scope() { active=_previous; }
        Scope(const Scope&)=delete;
        Scope& operator=(const Scope&)=delete;
    };
}
