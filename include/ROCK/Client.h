#pragma once
#include "Discovery.h"
#include "Core.h"
#include <cstring>

namespace rock::api {
    // Consumer-side helper only. It is not part of the DLL ABI and imports no
    // feature tables beyond Core. Pass the query export resolved from ROCK.dll.
    class Client {
        QueryInterfaceV1 _query{};
        const core::ApiV1* _core{};
        OwnerToken _owner{};
    public:
        Client()=default;
        Client(const Client&)=delete;
        Client& operator=(const Client&)=delete;
        // Close on the game thread before destroying callback userdata. Plugin
        // DLL unload with live callbacks is outside this process-lifetime API.
        Status connect(QueryInterfaceV1 query,const char* name) noexcept {
            if (_owner) return Status::Busy;
            if (!query || !name) return Status::InvalidArgument;
            core::RegistrationV1 registration{};
            std::size_t length=0;
            while (length<sizeof(registration.modName) && name[length]) ++length;
            if (!length || length==sizeof(registration.modName)) return Status::InvalidArgument;
            std::memcpy(registration.modName,name,length);
            const InterfaceDescriptorV1* descriptor{};
            auto status=query(core::kInterfaceId,1,0,sizeof(core::ApiV1),&descriptor);
            if (status!=Status::Ok) return status;
            if (!descriptor || descriptor->size!=sizeof(InterfaceDescriptorV1) ||
                descriptor->interfaceId!=core::kInterfaceId || descriptor->major!=1 ||
                descriptor->tableByteSize<sizeof(core::ApiV1) || !descriptor->table) return Status::InvalidSize;
            const auto* coreApi=static_cast<const core::ApiV1*>(descriptor->table);
            core::OwnerV1 owner{};
            status=coreApi->registerConsumerV1(&registration,&owner);
            if (status!=Status::Ok) return status;
            _query=query; _core=coreApi; _owner=owner.ownerToken;
            return Status::Ok;
        }
        template<class Table> Status acquire(std::uint32_t permissions,const Table*& output) const noexcept {
            output=nullptr;
            if (!_owner || !_query) return Status::OwnerNotRegistered;
            const InterfaceDescriptorV1* descriptor{};
            auto status=_query(Table::interfaceId,Table::majorVersion,Table::minorVersion,sizeof(Table),&descriptor);
            if (status!=Status::Ok) return status;
            if (!descriptor || descriptor->size!=sizeof(InterfaceDescriptorV1) ||
                descriptor->interfaceId!=Table::interfaceId || descriptor->major!=Table::majorVersion ||
                descriptor->minor<Table::minorVersion || descriptor->tableByteSize<sizeof(Table) ||
                descriptor->requiredCoreMajor!=core::kMajor || descriptor->requiredCoreMinor>core::kMinor || !descriptor->table) return Status::InvalidSize;
            status=_core->bindInterface(_owner,Table::interfaceId,Table::majorVersion,permissions);
            if (status==Status::Ok) output=static_cast<const Table*>(descriptor->table);
            return status;
        }
        Status close() noexcept {
            if (!_owner) return Status::Ok;
            const auto status=_core->unregisterConsumerV1(_owner);
            if (status==Status::Ok || status==Status::OwnerNotRegistered) {
                _owner=0; _core=nullptr; _query=nullptr;
            }
            return status;
        }
        [[nodiscard]] OwnerToken owner() const noexcept { return _owner; }
        [[nodiscard]] const core::ApiV1* core() const noexcept { return _core; }
    };
}
