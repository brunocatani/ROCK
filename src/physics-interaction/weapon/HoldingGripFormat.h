#pragma once
#include "physics-interaction/weapon/HoldingGripMath.h"
#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <stdexcept>
namespace rock::holding_grips {
struct FileRecord {std::string weapon,gripNode;bool left{},powerArmor{};holding_grip_math::Pose wrist;std::array<holding_grip_math::Pose,15> fingers;};
inline holding_grip_math::Pose readHoldingPose(const nlohmann::json& j){holding_grip_math::Pose p;p.r=j.at("rotate").get<std::array<float,9>>();p.t=j.at("translate").get<std::array<float,3>>();p.s=j.at("scale");if(!holding_grip_math::valid(p))throw std::runtime_error("Invalid holding transform");return p;}
inline std::vector<FileRecord> readHoldingGrips(const nlohmann::json& j){
 if(j.at("format")!="rock-holding-grips"||j.at("version")!=1||!j.at("grips").is_array())throw std::runtime_error("Unknown holding-grip format");std::vector<FileRecord> out;
 for(const auto& v:j.at("grips")){FileRecord x;x.weapon=v.at("weapon");x.gripNode=v.value("gripNode",std::string{});if(x.weapon.empty()||x.weapon.size()>256||x.weapon.find(':')==std::string::npos||x.gripNode.size()>127)throw std::runtime_error("Invalid holding grip identity");const auto hand=v.at("hand").get<std::string>();if(hand!="left"&&hand!="right")throw std::runtime_error("Invalid physical hand");x.left=hand=="left";x.powerArmor=v.value("powerArmor",false);x.wrist=readHoldingPose(v.at("wrist"));const auto& fingers=v.at("fingers");if(!fingers.is_array()||fingers.size()!=15)throw std::runtime_error("Expected fifteen finger locals");for(int i=0;i<15;++i)x.fingers[i]=readHoldingPose(fingers[i]);out.push_back(std::move(x));}return out;
}
}
