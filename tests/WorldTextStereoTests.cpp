#include "physics-interaction/debug/DebugWorldTextGeometry.h"
#include "physics-interaction/debug/DebugOverlayShaders.h"

#include <d3d11.h>
#include <d3dcompiler.h>
#include <wrl/client.h>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>

using Microsoft::WRL::ComPtr;
struct Point { float x, y, z; };
struct Vertex { Point p; std::array<float, 4> color; };
struct Camera { float matrices[2][16]{}; float origins[2][4]{}; };

void require(bool value, const char* reason) { if (!value) throw std::runtime_error(reason); }
void hr(HRESULT result, const char* reason) { if (FAILED(result)) { std::printf("HRESULT %08lX: %s\n", result, reason); throw std::runtime_error(reason); } }

int main()
{
    try {
        namespace geometry = rock::debug_world_text_geometry;
        const Point anchor{0.12f, 0.18f, 2}, right{1, 0, 0}, down{0, -1, 0};
        require(geometry::validBasis(anchor, right, down, 0.16f), "usable world panel rejected");
        require(!geometry::validBasis(anchor, right, right, 0.16f), "degenerate world panel accepted");
        const auto quad = geometry::pixelCorners(anchor, right, down, 0, 0, 0.16f);
        std::array<Vertex, 6> vertices{};
        constexpr unsigned corners[]{0, 1, 2, 0, 2, 3};
        for (unsigned i = 0; i < 6; ++i) vertices[i] = {quad[corners[i]], {1, 0, 1, 1}};

        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        hr(D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_WARP, nullptr, 0, nullptr, 0, D3D11_SDK_VERSION,
            &device, nullptr, &context), "create software D3D11 device");
        const auto compile = [](const char* source, const char* profile) {
            ComPtr<ID3DBlob> blob, error;
            const auto result = D3DCompile(source, std::strlen(source), nullptr, nullptr, nullptr, "main", profile,
                D3DCOMPILE_ENABLE_STRICTNESS | D3DCOMPILE_PACK_MATRIX_COLUMN_MAJOR, 0, &blob, &error);
            if (FAILED(result) && error) std::printf("%s\n", static_cast<const char*>(error->GetBufferPointer()));
            hr(result, "compile production overlay shader");
            return blob;
        };
        const auto vsCode = compile(rock::debug_overlay_shaders::kStereoColorVertex, "vs_5_0");
        const auto psCode = compile(rock::debug_overlay_shaders::kPixel, "ps_5_0");
        ComPtr<ID3D11VertexShader> vs;
        ComPtr<ID3D11PixelShader> ps;
        hr(device->CreateVertexShader(vsCode->GetBufferPointer(), vsCode->GetBufferSize(), nullptr, &vs), "vertex shader");
        hr(device->CreatePixelShader(psCode->GetBufferPointer(), psCode->GetBufferSize(), nullptr, &ps), "pixel shader");
        const D3D11_INPUT_ELEMENT_DESC elements[]{
            {"POS",0,DXGI_FORMAT_R32G32B32_FLOAT,0,0,D3D11_INPUT_PER_VERTEX_DATA,0},
            {"COLOR",0,DXGI_FORMAT_R32G32B32A32_FLOAT,0,12,D3D11_INPUT_PER_VERTEX_DATA,0}
        };
        ComPtr<ID3D11InputLayout> layout;
        hr(device->CreateInputLayout(elements, 2, vsCode->GetBufferPointer(), vsCode->GetBufferSize(), &layout), "input layout");
        Camera camera{};
        constexpr float sx[]{1.1f,0.9f}, sy[]{1.2f,1.05f}, skewX[]{0.12f,-0.08f}, skewY[]{0.015f,-0.025f};
        camera.origins[0][0] = -0.065f; camera.origins[1][0] = 0.065f;
        camera.origins[0][1] = 0.02f; camera.origins[1][1] = -0.01f;
        for (unsigned eye = 0; eye < 2; ++eye) {
            // Host row-major storage matches uploadCamera. HLSL reads this
            // as column-major: clip=(sx*x+skewX*z,sy*y+skewY*z,0.5*z,z).
            auto* m = camera.matrices[eye];
            m[0]=sx[eye]; m[5]=sy[eye]; m[8]=skewX[eye]; m[9]=skewY[eye]; m[10]=0.5f; m[11]=1;
        }
        const auto buffer = [&](UINT bytes, UINT bind, const void* data) {
            D3D11_BUFFER_DESC desc{}; desc.ByteWidth=bytes; desc.Usage=D3D11_USAGE_IMMUTABLE; desc.BindFlags=bind;
            D3D11_SUBRESOURCE_DATA initial{}; initial.pSysMem=data;
            ComPtr<ID3D11Buffer> result;
            hr(device->CreateBuffer(&desc,&initial,&result), "buffer"); return result;
        };
        auto vb=buffer(sizeof(vertices),D3D11_BIND_VERTEX_BUFFER,vertices.data());
        auto cb=buffer(sizeof(camera),D3D11_BIND_CONSTANT_BUFFER,&camera);
        constexpr UINT width=512, height=256;
        D3D11_TEXTURE2D_DESC td{}; td.Width=width; td.Height=height; td.MipLevels=1; td.ArraySize=1;
        td.Format=DXGI_FORMAT_R8G8B8A8_UNORM; td.SampleDesc.Count=1; td.BindFlags=D3D11_BIND_RENDER_TARGET;
        ComPtr<ID3D11Texture2D> target, readback;
        hr(device->CreateTexture2D(&td,nullptr,&target), "render target");
        td.Usage=D3D11_USAGE_STAGING; td.BindFlags=0; td.CPUAccessFlags=D3D11_CPU_ACCESS_READ;
        hr(device->CreateTexture2D(&td,nullptr,&readback), "readback");
        ComPtr<ID3D11RenderTargetView> rtv;
        hr(device->CreateRenderTargetView(target.Get(),nullptr,&rtv), "target view");
        D3D11_RASTERIZER_DESC rd{}; rd.FillMode=D3D11_FILL_SOLID; rd.CullMode=D3D11_CULL_NONE; rd.DepthClipEnable=TRUE;
        ComPtr<ID3D11RasterizerState> raster;
        hr(device->CreateRasterizerState(&rd,&raster), "raster state");
        const float black[4]{}; context->ClearRenderTargetView(rtv.Get(),black);
        auto* rawRtv=rtv.Get(); context->OMSetRenderTargets(1,&rawRtv,nullptr);
        const D3D11_VIEWPORT viewport{0,0,static_cast<float>(width),static_cast<float>(height),0,1};
        context->RSSetViewports(1,&viewport); context->RSSetState(raster.Get());
        context->IASetInputLayout(layout.Get()); context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        constexpr UINT stride=sizeof(Vertex), offset=0; auto* rawVb=vb.Get(); auto* rawCb=cb.Get();
        context->IASetVertexBuffers(0,1,&rawVb,&stride,&offset); context->VSSetConstantBuffers(0,1,&rawCb);
        context->VSSetShader(vs.Get(),nullptr,0); context->PSSetShader(ps.Get(),nullptr,0);
        context->DrawInstanced(6,2,0,0);
        context->CopyResource(readback.Get(),target.Get());
        D3D11_MAPPED_SUBRESOURCE mapped{}; hr(context->Map(readback.Get(),0,D3D11_MAP_READ,0,&mapped), "map result");
        std::array<double,2> sumX{},sumY{},count{};
        for (UINT y=0;y<height;++y) for (UINT x=0;x<width;++x) {
            const auto* pixel=static_cast<const unsigned char*>(mapped.pData)+y*mapped.RowPitch+x*4;
            if (pixel[0]>200 && pixel[2]>200) { const auto eye=x/(width/2); sumX[eye]+=x+0.5; sumY[eye]+=y+0.5; ++count[eye]; }
        }
        context->Unmap(readback.Get(),0);
        for (unsigned eye=0;eye<2;++eye) {
            require(count[eye]>0,"world glyph missing in one eye");
            const double expectedX=eye*256+(sx[eye]*(0.2-camera.origins[eye][0])/2+skewX[eye]+1)*128;
            const double expectedY=(1-(sy[eye]*(0.1-camera.origins[eye][1])/2+skewY[eye]))*128;
            std::printf("eye %u glyph center %.3f,%.3f expected %.3f,%.3f\n",eye,sumX[eye]/count[eye],sumY[eye]/count[eye],expectedX,expectedY);
            require(std::abs(sumX[eye]/count[eye]-expectedX)<1 && std::abs(sumY[eye]/count[eye]-expectedY)<1,
                "glyph does not follow that eye's projection");
        }
        require(std::abs(sumX[0]/count[0]-(sumX[1]/count[1]-256))>20,"test must distinguish correct projection from copied eye pixels");
        std::puts("Production stereo shader renders shared world text geometry correctly");
        return 0;
    } catch (const std::exception& error) { std::printf("FAIL %s\n",error.what()); return 1; }
}
