#include "physics-interaction/weapon/HoldingGripRecords.h"
#include "rock_support/Fo4VrRuntime.h"
#include "physics-interaction/PhysicsLog.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/NetImmerse/NiNode.h"
#include "physics-interaction/weapon/HoldingGripFormat.h"
#include <Windows.h>
#include <fstream>
#include <algorithm>
#include <format>
namespace rock::holding_grips {
namespace {
std::vector<Record> records;
std::string lower(std::string s){std::transform(s.begin(),s.end(),s.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});return s;}

}
// Startup only, before animation/interaction callbacks; records remain immutable.
void load(const std::filesystem::path& directory){try{
 records.clear();auto resolved=directory;
 if(resolved.is_relative()){wchar_t executable[32768]{};const auto size=GetModuleFileNameW(nullptr,executable,32768);if(size>0&&size<32768)resolved=std::filesystem::path(executable).parent_path()/resolved;}
 std::error_code ec;if(!std::filesystem::is_directory(resolved,ec))return;
 std::vector<std::filesystem::path> files;for(const auto& e:std::filesystem::directory_iterator(resolved))if(e.is_regular_file()&&e.path().extension()==".json")files.push_back(e.path());std::sort(files.begin(),files.end());
 for(const auto& f:files)try{
  std::ifstream in(f);nlohmann::json j;in>>j;
  for(const auto& v:readHoldingGrips(j)){Record x;x.weapon=lower(v.weapon);x.gripNode=v.gripNode;x.left=v.left;x.powerArmor=v.powerArmor;x.wrist=holding_grip_math::toNative(v.wrist);for(int i=0;i<15;++i)x.fingers.localTransforms[i]=holding_grip_math::toNative(v.fingers[i]);x.fingers.enabledMask=0x7fff;
   auto old=std::find_if(records.begin(),records.end(),[&](const auto& p){return p.weapon==x.weapon&&p.gripNode==x.gripNode&&p.left==x.left&&p.powerArmor==x.powerArmor;});if(old!=records.end())*old=std::move(x);else records.push_back(std::move(x));
  }
 }catch(const std::exception& e){ROCK_LOG_WARN(Animation,"Holding grip file {}: {}; native grips remain available",f.string(),e.what());}
 ROCK_LOG_INFO(Animation,"Loaded {} optional holding grips",records.size());
 }catch(const std::exception& e){ROCK_LOG_ERROR(Animation,"Holding grip directory could not be read: {}; native grips retained",e.what());}
}
const Record* find(const RE::TESObjectWEAP* weapon,const RE::NiAVObject* root,bool left,std::uint64_t generation,bool powerArmor){
 struct Cache {std::uintptr_t weapon{},root{};std::uint64_t generation{};bool powerArmor{};const Record* hand[2]{};};static Cache cache;
 if(cache.weapon==reinterpret_cast<std::uintptr_t>(weapon)&&cache.root==reinterpret_cast<std::uintptr_t>(root)&&cache.generation==generation&&cache.powerArmor==powerArmor)return cache.hand[left?0:1];
 if(records.empty()||!weapon)return nullptr;const auto* file=weapon->GetFile(0);if(!file)return nullptr;
 const auto key=lower(std::format("{}:{:06x}",file->GetFilename(),const_cast<RE::TESObjectWEAP*>(weapon)->GetLocalFormID()));
 auto* mount=root?f4vr::findNode(const_cast<RE::NiAVObject*>(root),"P-Grip"):nullptr;auto* child=mount?f4vr::getFirstChild(mount):nullptr;const std::string_view grip=child&&child->name.c_str()?child->name.c_str():"";
 cache={reinterpret_cast<std::uintptr_t>(weapon),reinterpret_cast<std::uintptr_t>(root),generation,powerArmor,{}};
 for(bool side:{true,false}){const Record* fallback=nullptr;for(const auto& r:records)if(r.weapon==key&&r.left==side&&r.powerArmor==powerArmor){if(!r.gripNode.empty()&&r.gripNode==grip){fallback=&r;break;}if(r.gripNode.empty())fallback=&r;}cache.hand[side?0:1]=fallback;}return cache.hand[left?0:1];
}
}
