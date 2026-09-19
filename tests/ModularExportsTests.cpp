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
    for (const auto name:std::array{"ROCKAPI_GetApi","ROCKAPI_GetProviderApi","ROCKAPI_GetDescriptorV1","GetROCKConfigurationApi"}) assert(!GetProcAddress(module,name));
    FreeLibrary(module);
}
