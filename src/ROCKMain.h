// Access to the F4SE messaging interface ROCK obtained at load (ROCKMain.cpp).
#pragma once

namespace F4SE
{
    class MessagingInterface;
}

namespace rock
{
    const F4SE::MessagingInterface* getROCKMessaging();
}
