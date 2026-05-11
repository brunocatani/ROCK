#include "physics-interaction/contact/ContactTargetIdentity.h"

#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/object/ObjectDetection.h"

#include "RE/Bethesda/FormComponents.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/Havok/hknpBody.h"
#include "RE/Havok/hknpBodyId.h"
#include "RE/Havok/hknpWorld.h"

#include <cmath>
#include <string_view>

namespace rock::contact_target_identity
{
    namespace
    {
        RE::NiPoint3 normalizeOrZero(const RE::NiPoint3& value)
        {
            const float lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z;
            if (!std::isfinite(lengthSquared) || lengthSquared <= 1.0e-6f) {
                return {};
            }

            const float inverseLength = 1.0f / std::sqrt(lengthSquared);
            return RE::NiPoint3(value.x * inverseLength, value.y * inverseLength, value.z * inverseLength);
        }

        std::string copyEditorId(RE::TESForm* form)
        {
            if (!form) {
                return {};
            }

            const char* editorId = form->GetFormEditorID();
            return editorId && editorId[0] ? std::string(editorId) : std::string{};
        }
    }

    ContactTargetIdentity resolveContactTarget(RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        std::uint32_t bodyId,
        contact_evidence::NativeContactEndpointKind endpointKind,
        const RE::NiPoint3* contactPointGame,
        const RE::NiPoint3* contactNormalGame,
        ContactTargetResolutionOptions options)
    {
        ContactTargetIdentity identity{};
        identity.bodyId = bodyId;
        identity.endpointKind = endpointKind;

        if (contactPointGame && isFinitePoint(*contactPointGame)) {
            identity.hasContactPoint = true;
            identity.contactPointGame = *contactPointGame;
        }
        if (contactNormalGame && isFinitePoint(*contactNormalGame)) {
            identity.hasContactNormal = true;
            identity.contactNormalGame = normalizeOrZero(*contactNormalGame);
            identity.surfaceHint = classifySurfaceHint(identity.contactNormalGame);
        }

        if (!hknpWorld || !contact_evidence::isValidBodyId(bodyId)) {
            identity.status = ContactTargetResolutionStatus::InvalidInput;
            return identity;
        }

        std::uint32_t filterInfo = 0;
        if (havok_runtime::tryReadFilterInfo(hknpWorld, RE::hknpBodyId{ bodyId }, filterInfo)) {
            identity.filterInfo = filterInfo;
            identity.layer = filterInfo & 0x7Fu;
        }

        auto* body = havok_runtime::getBody(hknpWorld, RE::hknpBodyId{ bodyId });
        if (!body) {
            identity.status = ContactTargetResolutionStatus::BodyUnreadable;
            return identity;
        }
        identity.valid = true;
        identity.motionIndex = body->motionIndex;
        identity.status = ContactTargetResolutionStatus::BodyResolved;

        if (!options.resolveReference) {
            return identity;
        }

        if (!bhkWorld) {
            identity.status = ContactTargetResolutionStatus::MissingBhkWorld;
            return identity;
        }

        auto* ref = resolveBodyToRef(bhkWorld, hknpWorld, RE::hknpBodyId{ bodyId });
        if (!ref) {
            identity.status = ContactTargetResolutionStatus::UnresolvedReference;
            return identity;
        }

        identity.status = ContactTargetResolutionStatus::ResolvedReference;
        identity.hasResolvedReference = true;
        identity.refFormId = ref->GetFormID();

        auto* baseForm = ref->GetObjectReference();
        if (!baseForm) {
            return identity;
        }

        identity.baseFormId = baseForm->GetFormID();

        if (options.includeRichText) {
            identity.hasRichText = true;
            identity.refEditorId = copyEditorId(ref);
            if (const char* typeName = baseForm->GetFormTypeString(); typeName && typeName[0]) {
                identity.formType = typeName;
            }
            if (const auto fullName = RE::TESFullName::GetFullName(*baseForm, false); !fullName.empty()) {
                identity.displayName = std::string(fullName);
            }
            identity.baseEditorId = copyEditorId(baseForm);
        }
        return identity;
    }
}
