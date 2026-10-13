#include "physics-interaction/weapon/HoldingGripFormat.h"
#include <cassert>
#include <fstream>
#include <iostream>
using namespace rock::holding_grips;using J=nlohmann::json;
int main(int argc,char** argv){
 const J pose={{"rotate",{1,0,0,0,1,0,0,0,1}},{"translate",{1,2,3}},{"scale",1}};
 J fingers=J::array();for(int i=0;i<15;++i)fingers.push_back(pose);
 J right={{"weapon","test.esp:000123"},{"hand","right"},{"gripNode","PistolGrip"},{"wrist",pose},{"fingers",fingers}};auto left=right;left["hand"]="left";left["wrist"]["translate"]={-4,5,6};
 J file={{"format","rock-holding-grips"},{"version",1},{"grips",{right,left}}};auto records=readHoldingGrips(file);assert(records.size()==2&&!records[0].left&&records[1].left&&records[0].wrist.t[0]==1&&records[1].wrist.t[0]==-4);
 auto bad=file;bad["grips"][1]["wrist"]["scale"]=-1;bool rejected=false;try{readHoldingGrips(bad);}catch(const std::exception&){rejected=true;}assert(rejected);
 if(argc==2){std::ifstream in(argv[1]);J actual;in>>actual;auto imported=readHoldingGrips(actual);assert(!imported.empty());std::cout<<"Imported "<<imported.size()<<" Maker firing grips through the ROCK loader codec\n";}
 std::cout<<"PASS independent hand records and invalid transform rejection\n";
}
