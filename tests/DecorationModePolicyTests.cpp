#include "physics-interaction/grab/DecorationModePolicy.h"
#include <cassert>

int main()
{
    using namespace rock::decoration_mode;
    assert(singleObject(0,0)==0);
    assert(singleObject(17,0)==17 && singleObject(0,17)==17);
    assert(singleObject(17,17)==17);
    assert(singleObject(17,18)==0);
    ClickState state;
    state.update(true,17,true,false,false,0);
    assert(state.reserved && !state.request);
    state.update(true,17,true,true,true,0);
    assert(state.request==17 && state.draining);
    state.update(true,17,true,true,false,0);
    assert(!state.request && state.reserved);
    // Anchoring releases the object. Its spent click must not open PALM.
    state.update(false,0,true,true,false,0);
    assert(!state.request && state.reserved);
    state.update(false,0,true,false,false,0);
    assert(state.reserved && !state.draining);
    state.update(false,0,true,false,false,0);
    assert(!state.reserved);
    // A press spent away from a supporting surface cannot anchor on arrival.
    state.update(true,0,true,true,true,0);
    state.update(true,17,true,true,false,0);
    assert(!state.request);
    state.update(true,17,true,false,false,0);
    state.update(true,17,true,false,true,0);
    assert(state.request==17); // A whole short click can fit between frames.
    state.update(true,17,false,true,true,0);
    state.update(true,17,true,true,true,0);
    assert(!state.request); // Menu/provider loss needs release to rearm.
    state.update(true,17,true,false,false,0);
    state.update(true,17,true,true,true,101);
    assert(!state.request && !state.reserved);
    state.update(false,17,true,false,false,0);
    state.update(false,17,true,true,true,0);
    assert(!state.request && !state.reserved);
    // Enabling the mode cannot adopt a click that began while it was off.
    state.update(true,17,true,false,true,0);
    assert(!state.request);
    state.update(true,17,true,true,true,0);
    assert(state.request==17);
}
