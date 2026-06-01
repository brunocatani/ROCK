#include "physics-interaction/hand/HandLifecycle.h"

#include <cstdio>
#include <string_view>

namespace
{
    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }
        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, bool value)
    {
        if (!value) {
            return true;
        }
        std::printf("%s expected false\n", label);
        return false;
    }

    bool expectReason(const char* label, const char* actual, std::string_view expected)
    {
        if (actual && actual == expected) {
            return true;
        }
        std::printf("%s expected reason %.*s got %s\n",
            label,
            static_cast<int>(expected.size()),
            expected.data(),
            actual ? actual : "(null)");
        return false;
    }

    bool expectNear(const char* label, float actual, float expected)
    {
        const float delta = actual > expected ? actual - expected : expected - actual;
        if (delta <= 0.0001f) {
            return true;
        }
        std::printf("%s expected %.4f got %.4f\n", label, expected, actual);
        return false;
    }
}

int main()
{
    using namespace rock::hand_semantic_contact_state;
    using rock::hand_collider_semantics::HandColliderRole;
    using rock::hand_collider_semantics::HandFinger;
    using rock::hand_collider_semantics::HandFingerSegment;

    bool ok = true;

    SemanticContactSet set{};
    SemanticContactRecord contact{};
    contact.valid = true;
    contact.role = HandColliderRole::IndexTip;
    contact.finger = HandFinger::Index;
    contact.segment = HandFingerSegment::Tip;
    contact.handBodyId = 10;
    contact.otherBodyId = 20;
    contact.hasContactPointGame = true;
    contact.contactPointGame = SemanticContactVector{ 1.0f, 2.0f, 3.0f };
    contact.hasContactNormalGame = true;
    contact.contactNormalGame = SemanticContactVector{ 0.0f, 0.0f, 1.0f };
    set.record(contact);

    const auto contacts = set.collectFreshForBody(20, 0);
    ok &= expectTrue("semantic contact collection preserves one fresh record", contacts.count == 1);
    const auto& stored = contacts.records[0];
    ok &= expectTrue("semantic contact preserves optional point flag", hasUsableContactPoint(stored));
    ok &= expectNear("semantic contact preserves point x", stored.contactPointGame.x, 1.0f);
    ok &= expectNear("semantic contact preserves point y", stored.contactPointGame.y, 2.0f);
    ok &= expectNear("semantic contact preserves point z", stored.contactPointGame.z, 3.0f);
    ok &= expectTrue("semantic contact preserves optional normal flag", hasUsableContactNormal(stored));

    auto decision = evaluateSemanticPivotCandidate(true, stored, 20, 0);
    ok &= expectTrue("fresh non-anchor semantic contact is a pivot candidate", decision.accept);
    ok &= expectReason("fresh semantic pivot reason", decision.reason, "semanticContact");

    set.advanceFrames();
    const auto stale = set.getFreshForRole(HandColliderRole::IndexTip, 10);
    decision = evaluateSemanticPivotCandidate(true, stale, 20, 0);
    ok &= expectFalse("stale semantic contact is rejected for pivot", decision.accept);
    ok &= expectReason("stale semantic pivot reason", decision.reason, "staleContact");

    SemanticContactRecord anchor = contact;
    anchor.role = HandColliderRole::PalmAnchor;
    anchor.framesSinceContact = 0;
    decision = evaluateSemanticPivotCandidate(true, anchor, 20, 0);
    ok &= expectFalse("anchor-only semantic contact is rejected", decision.accept);
    ok &= expectReason("anchor-only semantic pivot reason", decision.reason, "anchorOnly");

    return ok ? 0 : 1;
}
