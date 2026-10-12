#include "NativeInteractionState.h"
#include "physics-interaction/debug/DebugBodyOverlay.h"
#include <algorithm>
#include <cstring>

namespace rock::native_interaction_service {
    void appendVisuals(debug::BodyOverlayFrame& frame) {
        const auto& service=detail::state();
        if(reinterpret_cast<std::uintptr_t>(frame.world)!=service.frame.world)return;
        for(const auto& slot:service.extensions)if(slot.owner&&slot.visualFrame==service.frame.sample.frameIndex) {
            for(std::uint32_t i=0;i<slot.bodyCount&&frame.count<frame.entries.size();++i) {
                const auto& source=slot.bodies[i];auto& target=frame.entries[frame.count++];
                target.bodyId=RE::hknpBodyId{source.identity.body};target.role=debug::BodyOverlayRole::External;
                target.expectedCollisionObject=source.identity.collisionObject;target.expectedShape=source.identity.shape;
                target.visualFlags=source.flags;target.labelOffsetY=source.labelOffsetY;
                std::copy_n(source.color,4,target.color);
                if(source.flags&static_cast<unsigned>(api::native_interaction::VisualFlag::Dimensions))std::memcpy(target.label,source.label,sizeof(target.label));
            }

        }
    }
}
