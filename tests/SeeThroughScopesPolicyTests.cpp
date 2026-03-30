#include <cassert>

#include "physics-interaction/weapon/SeeThroughScopesPolicy.h"

int main()
{
    using namespace rock::see_through_scopes_policy;

    assert(isSeeThroughScopesPluginFilename("3dscopes-replacer.esp"));
    assert(isSeeThroughScopesPluginFilename("3dscopes-wmsr.esp"));
    assert(isSeeThroughScopesPluginFilename("3dscopes-m1a.esl"));
    assert(isSeeThroughScopesPluginFilename("3DSCOPES-REPLACER.ESM"));

    assert(!isSeeThroughScopesPluginFilename("AX50.esp"));
    assert(!isSeeThroughScopesPluginFilename("See-Through-Scopes.esp"));
    assert(!isSeeThroughScopesPluginFilename("my-3dscopes-replacer.esp"));
    assert(!isSeeThroughScopesPluginFilename("3dscopes-replacer.txt"));

    static_assert(kNativeScopeOverlayTarget == 48);
    static_assert(kBgsModPropertyBlockId == 1);
    static_assert(kDefaultReticleLookDotThreshold > 0.0f && kDefaultReticleLookDotThreshold < 1.0f);
    static_assert(kDefaultReticleDistanceThresholdGameUnits > 0.0f);
    static_assert(kDefaultReticleOffsetXGameUnits == 0.372727f);
    static_assert(kDefaultReticleOffsetZGameUnits == -0.149692f);
    static_assert(kCompatibilityConfigKey[0] != '\0');
    static_assert(kReticleAlignmentConfigKey[0] != '\0');

    return 0;
}
