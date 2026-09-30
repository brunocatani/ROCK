#include "physics-interaction/grab/HeldObjectTarget.h"
#include <cassert>
int main() {
    using namespace rock;using api::Status;
    held_object_target::Lease lease;api::grab::v1_3::HeldObjectTarget request;
    request.rootWorld.translate[0]=13;
    assert(lease.set(1,55,100,request)==Status::Ok);
    assert(lease.current(55,102)&&!lease.current(55,103)&&!lease.current(56,100));
    assert(lease.set(2,55,101,request)==Status::OwnerConflict);
    assert(lease.clear(2,55,101)==Status::OwnerConflict);
    assert(lease.set(2,55,103,request)==Status::Ok); // expiry frees ownership
    assert(lease.target.translate[0]==13);
    assert(lease.set(1,56,103,request)==Status::Ok); // exact new grab
    assert(lease.clear(1,56,103)==Status::Ok&&!lease.current(56,103));
    request.leaseFrames=3;assert(lease.set(1,56,104,request)==Status::InvalidArgument);
    request.leaseFrames=0;assert(lease.set(1,56,104,request)==Status::InvalidArgument);
}
