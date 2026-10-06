#include "api/StorageInteractionPolicy.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include <array>
#include <cassert>

int main()
{
    using namespace rock::storage_interaction_policy;
    assert(supports(1,48));
    for(unsigned major:{0u,2u,3u}) assert(!supports(major,48));
    for(unsigned size:{0u,8u,40u,47u,49u,64u}) assert(!supports(1,size));
    assert(!fresh(100,0) && !fresh(100,101) && !fresh(100,97));
    assert(fresh(100,98) && fresh(100,100));
    assert(validRouting(3,3,0) && validRouting(0,1,0));
    assert(!validRouting(4,3,0) && !validRouting(1,0,0) && !validRouting(1,4,0) && !validRouting(1,3,1));
    assert(allowsHand(1,false) && !allowsHand(1,true));
    assert(allowsHand(2,true) && !allowsHand(2,false));
    assert(!allowsHand(0,true) && !allowsHand(0,false));
    assert(exactHand(100,0x1234,100,0x1234));
    assert(!exactHand(100,0x9999,100,0x1234)); // recycled body ID
    assert(!exactHand(99,0x1234,100,0x1234)); // another object's body
    assert(!exactHand(100,0,100,0));
    assert(!exactHand(invalidBody,1,invalidBody,1));
    using namespace rock::collision_layer_policy;
    for(bool npc:{false,true}) {
        std::array<std::uint64_t,64> matrix{};
        applyRockGeneratedLayerPolicies(matrix.data(),true,false,false,npc);
        const auto mask=matrix[ROCK_LAYER_STORED_DISPLAY];
        const auto handMask=(std::uint64_t{1}<<ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY) |
            (std::uint64_t{1}<<ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY);
        assert(mask==handMask);
        for(unsigned layer=0;layer<64;++layer)
            assert(bool(matrix[layer] & (std::uint64_t{1}<<ROCK_LAYER_STORED_DISPLAY))==bool(handMask & (std::uint64_t{1}<<layer)));
        assert(isRockGeneratedColliderLayer(ROCK_LAYER_STORED_DISPLAY));
        assert(isRockOwnedMatrixLayer(ROCK_LAYER_STORED_DISPLAY));
    }
}
