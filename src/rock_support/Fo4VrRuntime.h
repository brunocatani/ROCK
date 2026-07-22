#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <RE/Fallout.h>
#include <REL/Relocation.h>

namespace rock::fo4vr
{
    inline constexpr std::uint32_t kPowerArmorKeywordFormId = 0x0004D8A1;
    inline constexpr std::uint32_t kPowerArmorFrameKeywordFormId = 0x0015503F;

    struct PlayerNodes
    {
        RE::NiNode* playerworldnode;                        // PlayerCharacter + 0x06E0
        RE::NiNode* roomnode;                               // + 0x06E8
        RE::NiNode* primaryWandNode;                        // + 0x06F0
        RE::NiNode* primaryWandandTouchPad;                 // + 0x06F8
        RE::NiNode* primaryUIAttachNode;                    // + 0x0700
        RE::NiNode* primaryWeapontoWeaponNode;              // + 0x0708
        RE::NiNode* primaryWeaponKickbackRecoilNode;        // + 0x0710
        RE::NiNode* primaryWeaponOffsetNOde;                // + 0x0718
        RE::NiNode* primaryWeaponScopeCamera;               // + 0x0720
        RE::NiNode* primaryVertibirdMinigunOffNOde;         // + 0x0728
        RE::NiNode* primaryMeleeWeaponOffsetNode;           // + 0x0730
        RE::NiNode* primaryUnarmedPowerArmorWeaponOffsetNode; // + 0x0738
        RE::NiNode* primaryWandLaserPointer;                // + 0x0740
        RE::NiNode* PrimaryWandLaserPointerAdjuster;        // + 0x0748
        RE::NiNode* unk750;                                 // + 0x0750
        RE::NiNode* PrimaryMeleeWeaponOffsetNode;           // + 0x0758
        RE::NiNode* SecondaryMeleeWeaponOffsetNode;         // + 0x0760
        RE::NiNode* SecondaryWandNode;                      // + 0x0768
        RE::NiNode* Point002Node;                           // + 0x0770
        RE::NiNode* WorkshopPalletNode;                     // + 0x0778
        RE::NiNode* WorkshopPallenSlide;                    // + 0x0780
        RE::NiNode* SecondaryUIOffsetNode;                  // + 0x0788
        RE::NiNode* SecondaryMeleeWeaponOffsetNode2;        // + 0x0790
        RE::NiNode* SecondaryUnarmedPowerArmorWeaponOffsetNode; // + 0x0798
        RE::NiNode* SecondaryAimNode;                       // + 0x07A0
        RE::NiNode* PipboyParentNode;                       // + 0x07A8
        RE::NiNode* PipboyRoot_nif_only_node;               // + 0x07B0
        RE::NiNode* ScreenNode;                             // + 0x07B8
        RE::NiNode* PipboyLightParentNode;                  // + 0x07C0
        RE::NiNode* unk7c8;                                 // + 0x07C8
        RE::NiNode* ScopeParentNode;                        // + 0x07D0
        RE::NiNode* unk7d8;                                 // + 0x07D8
        RE::NiNode* HmdNode;                                // + 0x07E0
        RE::NiNode* OffscreenHmdNode;                       // + 0x07E8
        RE::NiNode* UprightHmdNode;                         // + 0x07F0
        RE::NiNode* UprightHmdLagNode;                      // + 0x07F8
        RE::NiNode* BlackSphereNode;                        // + 0x0800
        RE::NiNode* HeadLightParentNode;                    // + 0x0808
        RE::NiNode* unk810;                                 // + 0x0810
        RE::NiNode* WeaponLeftNode;                         // + 0x0818
        RE::NiNode* unk820;                                 // + 0x0820
        RE::NiNode* LockPickParentNode;                     // + 0x0828
    };
    static_assert(sizeof(PlayerNodes) == 0x150);

    class BSFlattenedBoneTree : public RE::NiNode
    {
    public:
        struct BoneTransforms
        {
            RE::NiTransform local;
            RE::NiTransform world;
            std::int16_t parPos;
            std::int16_t childPos;
            std::uint32_t unk8c;
            RE::NiNode* refNode;
            RE::BSFixedString name;
            std::uint64_t unk98;
        };

        struct BoneNodePosition
        {
            RE::BSFixedString name;
            std::int32_t position;
            std::uint32_t pad;
            std::uintptr_t unk;
        };

        std::int32_t numTransforms;
        std::uint32_t pad0;
        BoneTransforms* transforms;
        std::uint64_t unk190;
        std::uint64_t unk198;
        std::uint64_t unk1a0;
        std::uint64_t unk1a8;
        std::uint64_t unk1b0;
        BoneNodePosition* bonePositions;
    };
    static_assert(sizeof(BSFlattenedBoneTree::BoneTransforms) == 0xA0);
    static_assert(sizeof(BSFlattenedBoneTree::BoneNodePosition) == 0x18);
    static_assert(sizeof(BSFlattenedBoneTree) == 0x1C0);

    struct NiCloneProcess
    {
        std::uint64_t unk00{ 0 };
        std::uint64_t unk08{ 0 };
        std::uint64_t unk10{ 0 };
        std::uint64_t* unk18{ nullptr };
        std::uint64_t unk20{ 0 };
        std::uint64_t unk28{ 0 };
        std::uint64_t unk30{ 0 };
        std::uint64_t unk38{ 0 };
        std::uint64_t unk40{ 0 };
        std::uint64_t* unk48{ nullptr };
        std::uint64_t unk50{ 0 };
        std::uint64_t unk58{ 0 };
        std::uint8_t copyType{ 1 };
        std::uint8_t affectedNodeRelationBehavior{ 0 };
        std::uint8_t dynamicEffectRelationBehavior{ 0 };
        char appendCharacter{ '$' };
        RE::NiPoint3 scale{ 1.0f, 1.0f, 1.0f };
    };
    static_assert(offsetof(NiCloneProcess, scale) == 0x64);
    static_assert(sizeof(NiCloneProcess) == 0x70);

    class MuzzleFlash
    {
    public:
        std::uint64_t unk00;
        std::uint64_t unk08;
        RE::NiNode* fireNode;
        RE::NiNode* projectileNode;
    };
    static_assert(sizeof(MuzzleFlash) == 0x20);

    using LoadNif = int (*)(std::uint64_t path, std::uint64_t output, std::uint64_t flags);
    inline REL::Relocation<LoadNif> loadNif{ REL::Offset(0x1D0DEE0) };

    using CloneNode = RE::NiNode* (*)(const RE::NiNode* node, NiCloneProcess* process);
    inline REL::Relocation<CloneNode> cloneNode{ REL::Offset(0x1C13FF0) };

    using IsActorUsingMelee = bool (*)(RE::Actor* actor);
    inline REL::Relocation<IsActorUsingMelee> CombatUtilities_IsActorUsingMelee{ REL::Offset(0x1133BB0) };

    inline REL::Relocation<std::uint64_t> EquippedWeaponData_vfunc{ REL::Offset(0x2D7FCF8) };
    inline REL::Relocation<std::uint64_t*> cloneAddr1{ REL::Offset(0x36FF560) };
    inline REL::Relocation<std::uint64_t*> cloneAddr2{ REL::Offset(0x36FF564) };

    [[nodiscard]] RE::PlayerCharacter* getPlayer() noexcept;
    [[nodiscard]] PlayerNodes* getPlayerNodes() noexcept;
    [[nodiscard]] RE::NiNode* getWorldRootNode() noexcept;
    [[nodiscard]] RE::NiNode* getRootNode() noexcept;
    [[nodiscard]] BSFlattenedBoneTree* getFlattenedBoneTree() noexcept;
    [[nodiscard]] RE::NiNode* getFirstPersonSkeleton() noexcept;
    [[nodiscard]] BSFlattenedBoneTree* getFirstPersonBoneTree() noexcept;
    [[nodiscard]] RE::EquippedItem* getEquippedItem() noexcept;
    [[nodiscard]] RE::EquippedWeaponData* getEquippedWeaponData() noexcept;
    [[nodiscard]] RE::NiNode* getWeaponNode() noexcept;
    [[nodiscard]] RE::PlayerCamera* getPlayerCamera() noexcept;
    [[nodiscard]] RE::NiPoint3 getCameraPosition() noexcept;
    [[nodiscard]] RE::NiNode* getLeftHandNode() noexcept;
    [[nodiscard]] RE::NiNode* getRightHandNode() noexcept;

    [[nodiscard]] bool IsWeaponDrawn() noexcept;
    [[nodiscard]] bool isMeleeWeaponEquipped() noexcept;
    [[nodiscard]] bool isInPowerArmor() noexcept;
    [[nodiscard]] RE::Setting* getIniSetting(const char* name) noexcept;

    void showNotification(const std::string& text);

    [[nodiscard]] RE::NiAVObject* getFirstChild(RE::NiAVObject* object) noexcept;
    [[nodiscard]] RE::NiNode* findNode(RE::NiAVObject* root, const char* name, int maxDepth = 999) noexcept;
    [[nodiscard]] RE::NiNode* find1StChildNode(RE::NiAVObject* root, const char* name) noexcept;
    [[nodiscard]] bool isNodeVisible(const RE::NiAVObject* node) noexcept;
    void updateDown(RE::NiAVObject* node, bool updateSelf, const char* ignoredNode = nullptr) noexcept;
    void updateTransforms(RE::NiAVObject* node) noexcept;
    void updateTransformsDown(RE::NiAVObject* node, bool updateSelf, const char* ignoredNode = nullptr) noexcept;

    [[nodiscard]] RE::NiNode* loadNifFromFile(const std::string& path);
}

namespace f4vr = rock::fo4vr;
