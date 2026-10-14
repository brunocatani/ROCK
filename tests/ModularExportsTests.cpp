#include <Windows.h>
#include <ROCK/Discovery.h>
#include <array>
#include <cassert>
int main(int argc,char** argv) {
    assert(argc==2);
    // Inspect only the PE export directory. CommonLib initializes against the
    // game executable, so a test host must not run the plugin's CRT startup.
    const auto module=LoadLibraryExA(argv[1],nullptr,DONT_RESOLVE_DLL_REFERENCES);
    assert(module);
    assert(GetProcAddress(module,rock::api::kQueryExportName));
    assert(GetProcAddress(module,"F4SEPlugin_Query"));
    assert(GetProcAddress(module,"F4SEPlugin_Load"));
    // Ejection contacts and object rest are part of Collision major 2; the
    // storage family is the Storage interface.
    for (const auto name:std::array{"ROCKAPI_GetApi","ROCKAPI_GetProviderApi","ROCKAPI_GetDescriptorV1","GetROCKConfigurationApi",
             "ROCK_ObjectRest_QueryV1","ROCK_EjectionContacts_QueryV1","ROCK_EjectionContacts_QueryV2",
             "ROCK_Storage_QueryV1","ROCK_StorageDisplay_QueryV1","ROCK_StorageFeedback_QueryV1","ROCK_StashConversion_QueryV1",
             "ROCK_StorageInteraction_QueryV1","ROCK_StorageInteraction_QueryV1_1","ROCK_InventoryDisplay_QueryV1","ROCK_InventoryDisplay_QueryV1_1",
             "ROCK_EquipPickup_QueryV1"}) assert(!GetProcAddress(module,name));
    FreeLibrary(module);
}
