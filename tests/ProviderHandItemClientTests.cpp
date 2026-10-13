#include "api/providers/Boundary.h"
#include <ROCK/Hands.h>
#include <cassert>
int main() {
    using namespace rock;
    api::hands::HandFrame pose{};
    pose.size=sizeof(pose)-1;
    assert(api::boundary::checkOutput(&pose)==api::Status::InvalidSize && pose.size==sizeof(pose)-1);
}
