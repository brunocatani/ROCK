#include "api/LooseWeaponStatePath.h"
#include "api/LooseWeaponStateRotation.h"
#include <cassert>
int main() {
    using rock::loose_weapon_state::relativeItemPath;
    constexpr auto path="Weapon  (0D01EC2E)/P-Receiver/WEAPON/WeaponExtra4";
    assert(relativeItemPath("Weapon",path)==path);
    assert(relativeItemPath("Weapon  (0D01EC2E)",path)=="P-Receiver/WEAPON/WeaponExtra4");
    assert(relativeItemPath("Weapon  (0D01EC2E)","Weapon  (0D01EC2E)").empty());
    assert(relativeItemPath("Weapon  (0D01EC2E)","Weapon  (0D01EC2E) extra/WeaponBolt")=="Weapon  (0D01EC2E) extra/WeaponBolt");
    assert(relativeItemPath("Weapon  (2400D8A5)",path)==path);
    assert(relativeItemPath("",path)==path);
    assert(relativeItemPath("WeaponMagazine","WeaponMagazine/WeaponMagazineChild1")=="WeaponMagazineChild1");
    // ReceiverParentObject replaces the equipped instance wrapper, while
    // the receiver attachment and its complete part chain remain below it.
    // M16A2E3 WEAP MODL: HandMade/Recievers/DummyReciever.nif. That NIF's
    // ReceiverParentObject owns CPA P-Receiver; the receiver has C-Receiver.
    constexpr auto receiverPath="Weapon  (2400D8A5)/P-Receiver/WeaponMagazine";
    assert(relativeItemPath("ReceiverParentObject",receiverPath,"Weapon  (2400D8A5)")=="P-Receiver/WeaponMagazine");
    assert(relativeItemPath("ReceiverParentObject","Weapon  (2400D8A5)/P-Receiver","Weapon  (2400D8A5)")=="P-Receiver");
    assert(relativeItemPath("ReceiverParentObject",receiverPath,"Weapon  (0D0087E8)")==receiverPath);
    assert(relativeItemPath("ReceiverParentObject","Weapon  (2400D8A5)/P-Mag/WeaponMagazine","Weapon  (2400D8A5)")=="P-Mag/WeaponMagazine");
    // Actual pistol root from the failed runtime session. Root names belong
    // to the base mesh, not to a hardcoded rifle/pistol family list.
    constexpr auto pistol="Weapon  (33000808)/P-Receiver/WeaponMagazine";
    assert(relativeItemPath("10mmReceiverParentObject",pistol,"Weapon  (33000808)")=="P-Receiver/WeaponMagazine");
    assert(relativeItemPath("CustomRoot",pistol,"Weapon  (33000808)")=="P-Receiver/WeaponMagazine");
    assert(relativeItemPath("Weapon",pistol,"Weapon  (33000808)")==pistol);

    // Exercise the production conversion against native 16-byte row stride.
    // A flat 36-byte memcpy corrupts even identity and collapses geometry.
    alignas(16) float native[3][4]={{1,0,0,101},{0,1,0,102},{0,0,1,103}};
    float packed[9]{};
    rock::loose_weapon_state::packRotation(packed,native);
    const float identity[9]={1,0,0,0,1,0,0,0,1};
    for(int i=0;i<9;++i)assert(packed[i]==identity[i]);
    const float quarterTurn[9]={0,-1,0,1,0,0,0,0,1};
    rock::loose_weapon_state::unpackRotation(native,quarterTurn);
    assert(native[0][0]==0&&native[0][1]==-1&&native[1][0]==1&&native[2][2]==1);
    assert(native[0][3]==101&&native[1][3]==102&&native[2][3]==103);
    rock::loose_weapon_state::packRotation(packed,native);
    for(int i=0;i<9;++i)assert(packed[i]==quarterTurn[i]);
}
