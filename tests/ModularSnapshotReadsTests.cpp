#include "api/providers/SnapshotReads.h"
#include "api/ProviderRuntimeServices.h"
#include "api/ProviderFrameThreadOwner.h"
#include <cassert>
#include <future>
#include <thread>

namespace {
    using rock::api::Status;
    const auto frameThread = std::this_thread::get_id();
    bool available = true;
    std::uint32_t permissions = 1;
    rock::provider::RockProviderFrameSnapshot publication{};
}

// Engine services are the fixture boundary. The production endpoint bodies,
// authorization invocation, marshaling and metadata extraction are linked.
namespace rock::provider::runtime {
    api::Status authorize(std::uint64_t owner, api::InterfaceId, std::uint32_t permission,
        bool requireThread, OwnerAccess) {
        if (owner != 42) return Status::OwnerNotRegistered;
        if ((permissions & permission) != permission) return Status::PermissionDenied;
        return requireThread && std::this_thread::get_id() != frameThread ? Status::WrongThread : Status::Ok;
    }
    void reportBoundaryFailure(std::uint64_t, api::InterfaceId) noexcept { assert(false); }
    bool ROCK_PROVIDER_CALL apiGetFrameSnapshot(RockProviderFrameSnapshot* out) {
        if (!available) return false;
        *out = publication;
        return true;
    }
    api::SampleV1 sample() {
        // A second read would observe a newer publication. The endpoint must
        // derive metadata from the payload it already copied, never this one.
        return {999,999,99,99,99,99};
    }
    bool ROCK_PROVIDER_CALL apiGetHandFrameV1(RockProviderHand hand, RockProviderHandFrameV1* out) {
        if (!available) return false;
        out->hand=hand;out->flags=1;out->frameIndex=100;out->worldGeneration=7;
        out->transform.translate[0]=12;
        return true;
    }
    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetEquippedWeaponStateV1(std::uint64_t, RockProviderEquippedWeaponStateV1* out) {
        if (!available) return RockProviderResultV1::NotReady;
        out->weaponFormId=123;out->weaponGenerationKey=456;out->frameIndex=100;
        return RockProviderResultV1::Ok;
    }
    bool ROCK_PROVIDER_CALL apiGetWeaponPartGripStateV1(RockProviderHand hand, api::weaponparts::WeaponPartGripStateV1* out) {
        if (!available) return false;
        out->hand=static_cast<api::Hand>(hand);out->active=1;out->sourceKey=789;out->weaponGenerationKey=456;
        return true;
    }
    void refreshSources() { assert(false); } // Task reads must never refresh live nodes.
    std::uint64_t sourceKey(std::uint64_t,std::uintptr_t) { assert(false);return 0; }
}

int main()
{
    using namespace rock;
    provider::ProviderFrameThreadOwner owner;
    assert(!owner.allows(11) && owner.owner()==0); // An early graph hook cannot bind ownership.
    assert(!owner.beginFrame(0));
    assert(owner.beginFrame(22) && owner.allows(22));
    assert(!owner.allows(11) && !owner.beginFrame(11) && owner.owner()==22);
    assert(owner.beginFrame(22));

    publication.frameIndex=100;publication.stateSequence=101;
    publication.worldGeneration=7;publication.skeletonGeneration=8;
    publication.providerGeneration=9;publication.collisionGeneration=10;
    publication.primaryHand=provider::RockProviderHand::Right;
    publication.offhandHand=provider::RockProviderHand::Left;
    publication.enrichmentFlags=6;publication.hmdTransform.translate[0]=12;
    publication.hmdForwardWorld[2]=1;publication.externalBodyCount=13;
    publication.gameToHavokScale=0.014f;publication.havokToGameScale=71;
    publication.weaponGenerationKey=456;publication.weaponBodyCount=1;publication.weaponBodyIds[0]=14;
    auto task=std::async(std::launch::async,[] {
        assert(std::this_thread::get_id()!=frameThread);
        api::hands::HeadPoseV1 head{};
        api::hands::RolesV1 roles{};
        api::collision::EnvironmentV1 environment{};
        assert(api::hands::getHeadPose(42,&head)==Status::Ok && head.valid==3 && head.transform.translate[0]==12);
        assert(api::hands::getRoles(42,&roles)==Status::Ok && roles.offhand==api::Hand::Left);
        assert(api::collision::getEnvironment(42,&environment)==Status::Ok && environment.weaponBodyIds[0]==14);
        for(const auto& sample:{head.sample,roles.sample,environment.sample}) {
            assert(sample.frameIndex==100 && sample.publicationSequence==101);
            assert(sample.worldGeneration==7 && sample.skeletonGeneration==8 && sample.providerGeneration==9 && sample.collisionGeneration==10);
        }
        api::weapon::EquippedWeaponStateV1 weapon{};
        assert(api::weapon::getEquippedWeaponStateV1(42,&weapon)==Status::Ok && weapon.weaponFormId==123 && weapon.frameIndex==100);
        for(const auto hand:{api::Hand::Right,api::Hand::Left}) {
            api::hands::HandFrameV1 frame{};
            api::weaponparts::WeaponPartGripStateV1 grip{};
            assert(api::hands::getHandFrameV1(42,hand,&frame)==Status::Ok && frame.hand==hand && frame.frameIndex==100);
            assert(api::weaponparts::getWeaponPartGripStateV1(42,hand,&grip)==Status::Ok && grip.hand==hand && grip.sourceKey==789);
        }
        assert(api::hands::getRoles(99,&roles)==Status::OwnerNotRegistered && roles.sample.frameIndex==0);
        permissions=0;
        assert(api::collision::getEnvironment(42,&environment)==Status::PermissionDenied && environment.weaponBodyCount==0);
        permissions=1;
        available=false;
        assert(api::hands::getHeadPose(42,&head)==Status::NotReady && head.valid==0);
        assert(api::weapon::getEquippedWeaponStateV1(42,&weapon)==Status::NotReady && weapon.weaponFormId==0);
        api::weaponparts::WeaponPartGripStateV1 grip{};grip.active=1;
        assert(api::weaponparts::getWeaponPartGripStateV1(42,api::Hand::Left,&grip)==Status::NotReady && grip.active==0);
    });
    task.get();
}
