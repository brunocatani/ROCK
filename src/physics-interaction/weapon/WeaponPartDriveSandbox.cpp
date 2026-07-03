#include "physics-interaction/weapon/WeaponPartDriveSandbox.h"

#include "api/ROCKProviderApi.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/weapon/WeaponPartMotionLearner.h"
#include "physics-interaction/weapon/WeaponPartMotionScrubPolicy.h"
#include "physics-interaction/weapon/WeaponTypes.h"

#include <algorithm>
#include <cstring>

namespace rock
{
    namespace
    {
        constexpr char kSandboxConsumerName[] = "ROCK.BoltDriveSandbox";
        constexpr std::uint32_t kRegistrationRetryFrames = 300;
        constexpr std::uint32_t kDriveLeaseFrames = 2;
        constexpr std::uint32_t kDrivePriority = 10;

        // Row-major 3x3 from a unit quaternion {w,x,y,z}; matches the
        // fillProviderTransform flattening consumed by providerTransformToNi.
        void quatToRotateRowMajor(const weapon_part_motion_path::Quat& q, float outRotate[9])
        {
            const auto n = weapon_part_motion_path::quatNormalizeOrIdentity(q);
            const float xx = n.x * n.x, yy = n.y * n.y, zz = n.z * n.z;
            const float xy = n.x * n.y, xz = n.x * n.z, yz = n.y * n.z;
            const float wx = n.w * n.x, wy = n.w * n.y, wz = n.w * n.z;
            outRotate[0] = 1.0f - 2.0f * (yy + zz);
            outRotate[1] = 2.0f * (xy - wz);
            outRotate[2] = 2.0f * (xz + wy);
            outRotate[3] = 2.0f * (xy + wz);
            outRotate[4] = 1.0f - 2.0f * (xx + zz);
            outRotate[5] = 2.0f * (yz - wx);
            outRotate[6] = 2.0f * (xz - wy);
            outRotate[7] = 2.0f * (yz + wx);
            outRotate[8] = 1.0f - 2.0f * (xx + yy);
        }

        std::string_view sessionName(const std::array<char, WeaponPartDriveSandbox::kMaxSourceName>& name)
        {
            std::size_t length = 0;
            while (length < name.size() && name[length] != '\0') {
                ++length;
            }
            return std::string_view(name.data(), length);
        }
    }

    bool WeaponPartDriveSandbox::ensureRegistered()
    {
        if (_ownerToken != 0 && _whitelistInstalled) {
            return true;
        }
        if (_registrationRetryCooldownFrames > 0) {
            --_registrationRetryCooldownFrames;
            return false;
        }

        const auto* api = ::rock::provider::ROCKAPI_GetProviderApi();
        if (!api || !api->registerConsumerV1 || !api->setWeaponPartTargetsV1) {
            return false;
        }

        if (_ownerToken == 0) {
            ::rock::provider::RockProviderConsumerRegistrationV1 registration{};
            std::memcpy(registration.modName, kSandboxConsumerName, sizeof(kSandboxConsumerName));
            registration.requestedCapabilities =
                static_cast<std::uint32_t>(::rock::provider::RockProviderConsumerCapabilityV1::WeaponPartInteraction);
            ::rock::provider::RockProviderConsumerHandleV1 handle{};
            const auto result = api->registerConsumerV1(&registration, &handle);
            if (result != ::rock::provider::RockProviderResultV1::Ok || handle.ownerToken == 0) {
                if (!_registrationWarned) {
                    ROCK_LOG_WARN(Weapon, "WeaponPartDriveSandbox: consumer registration failed result={}", static_cast<std::uint32_t>(result));
                    _registrationWarned = true;
                }
                _registrationRetryCooldownFrames = kRegistrationRetryFrames;
                return false;
            }
            _ownerToken = handle.ownerToken;
        }

        // Persistent whitelist: every Bolt-classified part is grabbable as an
        // AttachOnly glue on any weapon (generation key 0 = all generations).
        // NonExclusive keeps every other part grip on its normal behavior.
        ::rock::provider::RockProviderWeaponPartTargetV1 target{};
        target.flags =
            static_cast<std::uint32_t>(::rock::provider::RockProviderWeaponPartTargetFlagV1::MatchActionRole) |
            static_cast<std::uint32_t>(::rock::provider::RockProviderWeaponPartTargetFlagV1::NonExclusive);
        target.grabMode = ::rock::provider::RockProviderWeaponPartGrabModeV1::AttachOnly;
        target.actionRole = static_cast<std::uint32_t>(WeaponActionRole::Bolt);
        target.groupId = 1;
        target.priority = kDrivePriority;
        const auto targetResult = api->setWeaponPartTargetsV1(_ownerToken, &target, 1);
        if (targetResult != ::rock::provider::RockProviderResultV1::Ok) {
            if (!_registrationWarned) {
                ROCK_LOG_WARN(Weapon, "WeaponPartDriveSandbox: whitelist install failed result={}", static_cast<std::uint32_t>(targetResult));
                _registrationWarned = true;
            }
            _registrationRetryCooldownFrames = kRegistrationRetryFrames;
            return false;
        }
        _whitelistInstalled = true;
        ROCK_LOG_INFO(Weapon, "WeaponPartDriveSandbox: registered (token={}) with NonExclusive AttachOnly bolt whitelist", _ownerToken);
        return true;
    }

    void WeaponPartDriveSandbox::endSession(HandSession& session)
    {
        session = {};
    }

    void WeaponPartDriveSandbox::update(const FrameInput& input, const WeaponPartMotionLearner& learner)
    {
        if (!ensureRegistered()) {
            _sessions = {};
            _sentDrivesLastUpdate = false;
            return;
        }

        // Per hand: the gripped leader plus up to kMaxFollowers assembly parts.
        std::array<::rock::provider::RockProviderWeaponPartDriveTargetV1, 2 * (1 + weapon_clip_stroke::kMaxFollowers)> drives{};
        std::uint32_t driveCount = 0;

        for (std::size_t handIndex = 0; handIndex < 2; ++handIndex) {
            const auto& hand = input.hands[handIndex];
            auto& session = _sessions[handIndex];

            if (!hand.gripActive || input.weaponFormId == 0 || input.weaponGenerationKey == 0) {
                endSession(session);
                continue;
            }

            const bool freshGrip = !session.active ||
                session.gripSequence != hand.gripSequence ||
                session.bodyId != hand.bodyId ||
                session.weaponGenerationKey != input.weaponGenerationKey;
            if (freshGrip) {
                endSession(session);
                if (!hand.transformsValid || hand.sourceName.empty()) {
                    continue;
                }
                const auto group = learner.findGroup(input.weaponFormId, hand.sourceName);
                if (!group.leaderPath) {
                    if (_lastNoPathGripSequence[handIndex] != hand.gripSequence) {
                        _lastNoPathGripSequence[handIndex] = hand.gripSequence;
                        ROCK_LOG_INFO(Weapon,
                            "WeaponPartDriveSandbox: no motion path yet for part '{}' on weapon {:08X} — no authored clip stroke harvested and no runtime sample learned",
                            hand.sourceName,
                            input.weaponFormId);
                    }
                    continue;
                }
                const auto seeded = weapon_part_motion_scrub::initialScrubPosition(*group.leaderPath, hand.partTranslate);
                if (!seeded.valid) {
                    continue;
                }
                session.active = true;
                session.gripSequence = hand.gripSequence;
                session.bodyId = hand.bodyId;
                session.weaponGenerationKey = input.weaponGenerationKey;
                session.weaponFormId = input.weaponFormId;
                session.sourceName = {};
                std::memcpy(session.sourceName.data(), hand.sourceName.data(), (std::min)(hand.sourceName.size(), session.sourceName.size() - 1));
                session.arcPosition = seeded.arcPosition;
                session.handStartTranslate = hand.handTranslate;
                session.pathAnchorTranslate = seeded.target.translate;
                session.partScale = hand.partScale;
                session.followerCount = 0;
                if (group.authored && group.followers) {
                    session.followerCount = (std::min)(group.followerCount, static_cast<std::uint32_t>(session.followers.size()));
                    for (std::uint32_t i = 0; i < session.followerCount; ++i) {
                        session.followers[i] = group.followers[i];
                    }
                }
                ROCK_LOG_INFO(Weapon,
                    "WeaponPartDriveSandbox: scrub session started hand={} part='{}' arc={:.2f}/{:.2f} source={} followers={}",
                    handIndex == 1 ? "left" : "right",
                    hand.sourceName,
                    seeded.arcPosition,
                    group.leaderPath->totalArcLength,
                    group.authored ? "authored-clip" : "runtime-learned",
                    session.followerCount);
            }

            if (!session.active || !hand.transformsValid) {
                continue;
            }
            const auto* path = learner.findPath(session.weaponFormId, sessionName(session.sourceName));
            if (!path) {
                endSession(session);
                continue;
            }

            const weapon_part_motion_path::Vec3 desired{
                session.pathAnchorTranslate.x + (hand.handTranslate.x - session.handStartTranslate.x),
                session.pathAnchorTranslate.y + (hand.handTranslate.y - session.handStartTranslate.y),
                session.pathAnchorTranslate.z + (hand.handTranslate.z - session.handStartTranslate.z),
            };
            const auto scrubbed = weapon_part_motion_scrub::scrub(*path, session.arcPosition, desired);
            if (!scrubbed.valid) {
                continue;
            }
            session.arcPosition = scrubbed.arcPosition;

            // One drive per node: if both hands somehow grip the same body,
            // the first (right) hand keeps authority for this frame.
            bool duplicateBody = false;
            for (std::uint32_t i = 0; i < driveCount; ++i) {
                if (drives[i].bodyId == session.bodyId) {
                    duplicateBody = true;
                    break;
                }
            }
            if (duplicateBody) {
                continue;
            }

            auto& drive = drives[driveCount++];
            drive.flags = static_cast<std::uint32_t>(::rock::provider::RockProviderWeaponPartTargetFlagV1::MatchBodyId);
            drive.driveSpace = ::rock::provider::RockProviderWeaponPartDriveSpaceV1::WeaponRootLocal;
            drive.weaponGenerationKey = session.weaponGenerationKey;
            drive.bodyId = session.bodyId;
            drive.groupId = static_cast<std::uint32_t>(handIndex + 1);
            drive.priority = kDrivePriority;
            drive.leaseFrames = kDriveLeaseFrames;
            quatToRotateRowMajor(scrubbed.target.rotate, drive.targetTransform.rotate);
            drive.targetTransform.translate[0] = scrubbed.target.translate.x;
            drive.targetTransform.translate[1] = scrubbed.target.translate.y;
            drive.targetTransform.translate[2] = scrubbed.target.translate.z;
            drive.targetTransform.scale = session.partScale;

            // Authored assembly followers move at the same stroke progress —
            // driven by source name so they need no collider evidence.
            const float keyPosition = weapon_clip_stroke::keyPositionForArc(*path, session.arcPosition);
            for (std::uint32_t i = 0; i < session.followerCount && driveCount < drives.size(); ++i) {
                const auto& follower = session.followers[i];
                if (follower.boneName[0] == '\0') {
                    continue;
                }
                const auto followerPose = weapon_clip_stroke::followerPoseAtKeyPosition(follower, keyPosition);
                auto& followerDrive = drives[driveCount++];
                followerDrive.flags = static_cast<std::uint32_t>(::rock::provider::RockProviderWeaponPartTargetFlagV1::MatchSourceName);
                followerDrive.driveSpace = ::rock::provider::RockProviderWeaponPartDriveSpaceV1::WeaponRootLocal;
                followerDrive.weaponGenerationKey = session.weaponGenerationKey;
                followerDrive.groupId = static_cast<std::uint32_t>(handIndex + 1);
                followerDrive.priority = kDrivePriority;
                followerDrive.leaseFrames = kDriveLeaseFrames;
                std::memcpy(
                    followerDrive.sourceName,
                    follower.boneName.data(),
                    (std::min)(follower.boneName.size(), sizeof(followerDrive.sourceName) - 1));
                quatToRotateRowMajor(followerPose.rotate, followerDrive.targetTransform.rotate);
                followerDrive.targetTransform.translate[0] = followerPose.translate.x;
                followerDrive.targetTransform.translate[1] = followerPose.translate.y;
                followerDrive.targetTransform.translate[2] = followerPose.translate.z;
                followerDrive.targetTransform.scale = follower.restScale;
            }
        }

        const auto* api = ::rock::provider::ROCKAPI_GetProviderApi();
        if (!api || !api->setWeaponPartDriveTargetsV1 || !api->clearWeaponPartDriveTargetsV1) {
            return;
        }
        if (driveCount > 0) {
            const auto result = api->setWeaponPartDriveTargetsV1(_ownerToken, drives.data(), driveCount);
            _sentDrivesLastUpdate = result == ::rock::provider::RockProviderResultV1::Ok;
        } else if (_sentDrivesLastUpdate) {
            (void)api->clearWeaponPartDriveTargetsV1(_ownerToken);
            _sentDrivesLastUpdate = false;
        }
    }

    void WeaponPartDriveSandbox::shutdown()
    {
        if (_ownerToken != 0) {
            const auto* api = ::rock::provider::ROCKAPI_GetProviderApi();
            if (api && api->unregisterConsumerV1) {
                // Unregistration also clears this owner's whitelist targets
                // and drive leases inside the provider store.
                (void)api->unregisterConsumerV1(_ownerToken);
            }
        }
        _ownerToken = 0;
        _whitelistInstalled = false;
        _sentDrivesLastUpdate = false;
        _registrationRetryCooldownFrames = 0;
        _registrationWarned = false;
        _sessions = {};
        _lastNoPathGripSequence = {};
    }
}
