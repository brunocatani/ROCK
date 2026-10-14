#pragma once
#include "physics-interaction/grab/GrabEvent.h"
#include <ROCK/Grab.h>
namespace rock::provider::grab_callbacks {
    // One Grab event callback per owner; ROCK calls it on the producing
    // thread as each grab event happens.
    api::Status set(api::OwnerToken,api::grab::EventCallback,void*);
    void clear(api::OwnerToken);
    // Delivers the event to every callback. A faulting callback is cleared
    // and its owner revoked after delivery.
    void deliver(const rock::GrabEventData&,const api::Sample&);
}
