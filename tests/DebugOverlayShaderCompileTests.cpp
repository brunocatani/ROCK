#include "physics-interaction/debug/DebugOverlayShaders.h"

#include <cstddef>
#include <iostream>

#include <d3dcompiler.h>
#include <wrl/client.h>

namespace
{
    bool compileShader(const char* source, std::size_t sourceSize, const char* name, const char* target)
    {
        Microsoft::WRL::ComPtr<ID3DBlob> bytecode;
        Microsoft::WRL::ComPtr<ID3DBlob> errors;
        const HRESULT result = D3DCompile(
            source,
            sourceSize,
            name,
            nullptr,
            nullptr,
            "main",
            target,
            D3DCOMPILE_ENABLE_STRICTNESS | D3DCOMPILE_PACK_MATRIX_COLUMN_MAJOR,
            0,
            bytecode.GetAddressOf(),
            errors.GetAddressOf());

        if (FAILED(result)) {
            std::cerr << name << " failed to compile";
            if (errors) {
                std::cerr << ": " << static_cast<const char*>(errors->GetBufferPointer());
            }
            std::cerr << '\n';
        }
        return SUCCEEDED(result);
    }
}

int main()
{
    using namespace rock::debug_overlay_shaders;
    const bool bodyVertex = compileShader(kInstancedBodyVertex, sizeof(kInstancedBodyVertex) - 1, "ROCKDebugBodyVS", "vs_5_0");
    const bool stereoColorVertex = compileShader(kStereoColorVertex, sizeof(kStereoColorVertex) - 1, "ROCKDebugColorVS", "vs_5_0");
    const bool textVertex = compileShader(kScreenTextVertex, sizeof(kScreenTextVertex) - 1, "ROCKDebugTextVS", "vs_5_0");
    const bool pixel = compileShader(kPixel, sizeof(kPixel) - 1, "ROCKDebugBodyPS", "ps_5_0");
    return bodyVertex && stereoColorVertex && textVertex && pixel ? 0 : 1;
}
