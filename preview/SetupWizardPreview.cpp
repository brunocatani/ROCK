/*
 * ROCK Setup Wizard Preview: the setup wizard on the desktop, without the game.
 *
 * The panel is ROCK's own (src/setup-wizard/SetupWizardPanel.cpp), run by RPS UI's
 * LocalPanelHost exactly as RPS UI runs it in the headset, with RPS UI's default
 * theme and faces. Settings go through ROCK's real settings store over a sandbox
 * folder, so saving writes a real ROCK.toml there and never the game's. What only
 * the game has is stood in for on the right: what SteamVR reports and installs, and
 * the grip tap that goes Back.
 */

#include <Windows.h>
#include <ShlObj.h>
#include <shellapi.h>
#include <d3d11.h>
#include <dxgi.h>
#include <wincodec.h>
#include <wrl/client.h>

#include <RPS_UI_Framework/ImGuiPanelHost.h>

#include <imgui.h>
#include <imgui_impl_dx11.h>
#include <imgui_impl_win32.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "PreviewBackend.h"
#include "setup-wizard/SetupWizardPanel.h"
#include "setup-wizard/SetupWizardSettings.h"
#include "setup-wizard/SteamVrBindingPolicy.h"

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND, UINT, WPARAM, LPARAM);

namespace
{
    using Microsoft::WRL::ComPtr;
    namespace fs = std::filesystem;
    namespace fw = rpsui::api::framework;
    namespace sv = rock::steamvr_binding_policy;
    namespace wizard = rock::setup_wizard;
    namespace preview = rock::setup_wizard::preview;
    namespace wp = rock::setup_wizard_policy;

    // ---- window and device ---------------------------------------------------------------------

    ComPtr<ID3D11Device> g_device;
    ComPtr<ID3D11DeviceContext> g_context;
    ComPtr<IDXGISwapChain> g_swap;
    ComPtr<ID3D11RenderTargetView> g_backBuffer;
    UINT g_resizeWidth = 0;
    UINT g_resizeHeight = 0;
    ImGuiContext* g_harness = nullptr;

    bool createBackBuffer()
    {
        ComPtr<ID3D11Texture2D> back;
        return SUCCEEDED(g_swap->GetBuffer(0, IID_PPV_ARGS(&back))) &&
            SUCCEEDED(g_device->CreateRenderTargetView(back.Get(), nullptr, &g_backBuffer));
    }

    LRESULT CALLBACK windowProc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
    {
        // The panel host leaves its own context current; messages belong to the preview's.
        if (g_harness) ImGui::SetCurrentContext(g_harness);
        if (g_harness && ImGui_ImplWin32_WndProcHandler(hwnd, message, wparam, lparam)) return true;
        switch (message) {
        case WM_SIZE:
            if (wparam != SIZE_MINIMIZED) {
                g_resizeWidth = LOWORD(lparam);
                g_resizeHeight = HIWORD(lparam);
            }
            return 0;
        case WM_SYSCOMMAND:
            if ((wparam & 0xfff0) == SC_KEYMENU) return 0;
            break;
        case WM_DESTROY: PostQuitMessage(0); return 0;
        default: break;
        }
        return DefWindowProcW(hwnd, message, wparam, lparam);
    }

    // The panel's render target, sized to the pixels it is shown at.
    struct PanelTarget
    {
        ComPtr<ID3D11Texture2D> texture;
        ComPtr<ID3D11RenderTargetView> target;
        ComPtr<ID3D11ShaderResourceView> view;
        UINT width = 0;
        UINT height = 0;

        bool ensure(UINT w, UINT h)
        {
            w = (std::max)(w, 64u);
            h = (std::max)(h, 64u);
            if (texture && w == width && h == height) return true;
            view.Reset();
            target.Reset();
            texture.Reset();
            D3D11_TEXTURE2D_DESC desc{};
            desc.Width = w;
            desc.Height = h;
            desc.MipLevels = 1;
            desc.ArraySize = 1;
            desc.Format = DXGI_FORMAT_B8G8R8A8_UNORM;
            desc.SampleDesc.Count = 1;
            desc.BindFlags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE;
            if (FAILED(g_device->CreateTexture2D(&desc, nullptr, &texture)) ||
                FAILED(g_device->CreateRenderTargetView(texture.Get(), nullptr, &target)) ||
                FAILED(g_device->CreateShaderResourceView(texture.Get(), nullptr, &view)))
                return false;
            width = w;
            height = h;
            return true;
        }
    };

    // ---- files -----------------------------------------------------------------------------------

    [[nodiscard]] fs::path knownFolder(REFKNOWNFOLDERID id)
    {
        PWSTR path = nullptr;
        fs::path result;
        if (SUCCEEDED(SHGetKnownFolderPath(id, 0, nullptr, &path))) result = path;
        CoTaskMemFree(path);
        return result;
    }

    [[nodiscard]] fs::path gameModsConfig() { return knownFolder(FOLDERID_Documents) / "My Games" / "Fallout4VR" / "Mods_Config"; }

    [[nodiscard]] std::vector<char> readFile(const fs::path& path)
    {
        std::ifstream file(path, std::ios::binary);
        if (!file) return {};
        return { std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>() };
    }

    // RPS UI's faces, looked up as RPS UI looks them up: its Fonts folder, then Windows' fonts.
    struct Faces
    {
        std::array<std::vector<char>, fw::kFontRoleCount> bytes;

        void load()
        {
            const auto own = gameModsConfig() / "RPS_UI_Framework" / "Fonts";
            const auto windows = knownFolder(FOLDERID_Fonts);
            for (std::size_t role = 0; role < bytes.size(); ++role) {
                const fs::path file{ std::string(rpsui::theme_defaults::kFonts[role].file) };
                bytes[role] = readFile(own / file);
                if (bytes[role].empty()) bytes[role] = readFile(windows / file);
            }
        }

        [[nodiscard]] rpsui::imgui_host::FaceSet set() const
        {
            rpsui::imgui_host::FaceSet faces{};
            for (std::size_t role = 0; role < bytes.size(); ++role) faces[role] = { bytes[role].data(), bytes[role].size() };
            return faces;
        }
    };

    void openInShell(const fs::path& path)
    {
        const auto result = reinterpret_cast<INT_PTR>(ShellExecuteW(nullptr, L"open", path.c_str(), nullptr, nullptr, SW_SHOWNORMAL));
        if (result <= 32 && !fs::is_directory(path))
            (void)ShellExecuteW(nullptr, L"open", L"notepad.exe", path.c_str(), nullptr, SW_SHOWNORMAL);
    }

    // ---- what SteamVR would report ---------------------------------------------------------------

    enum class Binding : int
    {
        Default,
        Rock,
        PlayersOwn,
    };

    // The controller types SteamVR may report, as Prop_ControllerType_String.
    struct Reported
    {
        const char* label;
        const char* type;
    };

    constexpr std::array kReported{
        Reported{ "Index controllers", "knuckles" },
        Reported{ "Vive wands", "vive_controller" },
        Reported{ "Touch (Index or wands too, under ROCK's binding)", "oculus_touch" },
        Reported{ "Steam Frame", "frame_controller" },
        Reported{ "Other (Windows Mixed Reality)", "holographic_controller" },
        Reported{ "None turned on", "" },
    };

    struct SteamVrScenario
    {
        bool openComposite{ false };
        int reported{ 0 };
        Binding index{ Binding::Default };
        Binding vive{ Binding::Default };
        bool applySucceeds{ true };
        std::array<char, 160> applyError{};

        [[nodiscard]] rock::steamvr_binding::Inspection inspection(std::string_view type, Binding binding,
            const fs::path& settings) const
        {
            rock::steamvr_binding::Inspection result{};
            result.controller = sv::controllerFor(type);
            result.appKey = "preview";
            if (binding == Binding::Rock)
                result.currentUrl = sv::fileUrl((settings / "SteamVR" / std::string(result.controller->fileName)).string());
            else if (binding == Binding::PlayersOwn)
                result.currentUrl = "file:///C:/Users/Player/Documents/steamvr/my_fallout4vr_binding.json";
            result.status = sv::classify(true, result.controller, result.currentUrl);
            return result;
        }

        // What steamvr_binding::read() would return.
        [[nodiscard]] rock::steamvr_binding::State read(const fs::path& settings) const
        {
            rock::steamvr_binding::State state{};
            if (openComposite) return state;
            state.steamVr = true;
            state.reportedType = kReported[static_cast<std::size_t>(reported)].type;
            state.index = inspection("knuckles", index, settings);
            state.vive = inspection("vive_controller", vive, settings);
            return state;
        }
    };
    // ---- the preview's own look -----------------------------------------------------------------

    [[nodiscard]] fw::Theme harnessTheme(float scale)
    {
        auto theme = rpsui::theme_defaults::robco();
        theme.fontSize = 15.0f * scale;
        theme.titleFontSize = 18.0f * scale;
        theme.smallFontSize = 13.0f * scale;
        theme.targetHeight = 26.0f * scale;
        theme.padding = 6.0f * scale;
        theme.spacing = 6.0f * scale;
        return theme;
    }

    void muted(const fw::Theme& theme, const char* text)
    {
        ImGui::PushStyleColor(ImGuiCol_Text, rpsui::imgui_host::color(theme.textMuted));
        ImGui::TextWrapped("%s", text);
        ImGui::PopStyleColor();
    }

    [[nodiscard]] fs::path executableFolder()
    {
        std::wstring path(MAX_PATH, L'\0');
        const auto length = GetModuleFileNameW(nullptr, path.data(), static_cast<DWORD>(path.size()));
        path.resize(length);
        return fs::path(path).parent_path();
    }

    // The value after `name` on the command line, or empty.
    [[nodiscard]] fs::path argument(std::wstring_view name)
    {
        int count = 0;
        auto** arguments = CommandLineToArgvW(GetCommandLineW(), &count);
        fs::path value;
        for (int i = 1; arguments && i + 1 < count; ++i)
            if (std::wstring_view(arguments[i]) == name) value = arguments[i + 1];
        LocalFree(arguments);
        return value;
    }

    // --settings <folder> picks the sandbox; otherwise "Settings sandbox" beside the exe,
    // so a copy of the preview keeps its own.
    [[nodiscard]] fs::path sandboxFolder()
    {
        auto folder = argument(L"--settings");
        if (folder.empty()) folder = executableFolder() / "Settings sandbox";
        return fs::absolute(folder);
    }

    // The panel registered as ROCK registers it with RPS UI.
    [[nodiscard]] fw::PanelRegistration registration(wizard::WizardPanel& panel)
    {
        fw::PanelRegistration value{};
        std::snprintf(value.panelId, sizeof(value.panelId), "%s", "rock.setup.wizard");
        std::snprintf(value.displayName, sizeof(value.displayName), "%s", "ROCK Setup");
        value.layoutWidth = wizard::kLayoutWidth;
        value.layoutHeight = wizard::kLayoutHeight;
        value.imgui = RPSUI_IMGUI_BUILD();
        value.setupCallback = &wizard::WizardPanel::setupCallback;
        value.drawCallback = &wizard::WizardPanel::drawCallback;
        value.userData = &panel;
        return value;
    }

    // The game's side of the wizard, in the game's order: opening it, the settings store and
    // SteamVR.
    struct GameSide
    {
        wizard::WizardPanel panel;
        SteamVrScenario steamVr;
        std::optional<wizard::FinishResult> lastFinish;

        GameSide()
        {
            std::snprintf(steamVr.applyError.data(), steamVr.applyError.size(), "%s",
                "SteamVR refused the setting (VRSettingsError 2)");
        }

        // As the game opens it: a fresh session from the settings as saved, and SteamVR as it is now.
        void open()
        {
            preview::reloadSettings();
            auto start = wizard::settings::read();
            start.steamVr = steamVr.read(preview::settingsDirectory());
            panel.begin(std::move(start));
            panel.clearRequests();
            panel.setOpen(true);
        }

        void close()
        {
            panel.setOpen(false);
            panel.clearRequests();
        }

        // One frame before the panel draws: what the player asked for, done as the game's host does.
        void service()
        {
            if (!panel.isOpen()) return;
            const auto requests = panel.takeRequests();
            bool keepOpen = !requests.close;
            if (requests.finish) {
                wizard::FinishResult result{};
                wizard::settings::write(requests.writes, result);
                if (requests.binding.action == wp::BindingAction::Install) {
                    if (steamVr.applySucceeds) {
                        result.bindingInstalled = true;
                        (*requests.binding.controller == sv::Controller::Index ? steamVr.index : steamVr.vive) = Binding::Rock;
                    } else {
                        result.bindingError = steamVr.applyError.data();
                    }
                }
                lastFinish = result;
                keepOpen = keepOpen && panel.finished(*requests.finish, result);
            }
            if (!keepOpen) close();
        }
    };
    // ---- --capture: every page to PNG, offscreen -------------------------------------------------

    void check(HRESULT result, const char* what)
    {
        if (FAILED(result)) throw std::runtime_error(std::string(what) + " failed");
    }

    void writePng(ID3D11Texture2D* texture, UINT width, UINT height, const fs::path& file)
    {
        D3D11_TEXTURE2D_DESC desc{};
        texture->GetDesc(&desc);
        desc.BindFlags = 0;
        desc.Usage = D3D11_USAGE_STAGING;
        desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
        ComPtr<ID3D11Texture2D> staging;
        check(g_device->CreateTexture2D(&desc, nullptr, &staging), "Staging texture");
        g_context->CopyResource(staging.Get(), texture);
        D3D11_MAPPED_SUBRESOURCE mapped{};
        check(g_context->Map(staging.Get(), 0, D3D11_MAP_READ, 0, &mapped), "Texture map");
        ComPtr<IWICImagingFactory> factory;
        check(CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&factory)), "WIC");
        ComPtr<IWICStream> stream;
        check(factory->CreateStream(&stream), "WIC stream");
        check(stream->InitializeFromFilename(file.c_str(), GENERIC_WRITE), "PNG file");
        ComPtr<IWICBitmapEncoder> encoder;
        check(factory->CreateEncoder(GUID_ContainerFormatPng, nullptr, &encoder), "PNG encoder");
        check(encoder->Initialize(stream.Get(), WICBitmapEncoderNoCache), "PNG encoder");
        ComPtr<IWICBitmapFrameEncode> frame;
        check(encoder->CreateNewFrame(&frame, nullptr), "PNG frame");
        check(frame->Initialize(nullptr), "PNG frame");
        check(frame->SetSize(width, height), "PNG size");
        WICPixelFormatGUID format = GUID_WICPixelFormat32bppBGRA;
        check(frame->SetPixelFormat(&format), "PNG format");
        check(frame->WritePixels(height, mapped.RowPitch, mapped.RowPitch * height, static_cast<BYTE*>(mapped.pData)), "PNG pixels");
        check(frame->Commit(), "PNG frame");
        check(encoder->Commit(), "PNG file");
        g_context->Unmap(staging.Get(), 0);
    }

    /*
     * Walks the wizard the way a player does, clicking where its buttons are at RPS UI's
     * default theme, and saves each page. The walk saves into the sandbox like any other run.
     */
    int capture(const fs::path& folder, const Faces& faces)
    {
        fs::create_directories(folder);
        GameSide game;
        PanelTarget target;
        if (!target.ensure(static_cast<UINT>(wizard::kLayoutWidth), static_cast<UINT>(wizard::kLayoutHeight))) return 1;
        rpsui::imgui_host::LocalPanelHost host(g_device.Get(), g_context.Get(), registration(game.panel), faces.set());

        int index = 0;
        const auto frame = [&](rpsui::imgui_host::PanelInput input) {
            game.service();
            const float clear[4]{ 0.0f, 0.0f, 0.0f, 0.0f };
            g_context->ClearRenderTargetView(target.target.Get(), clear);
            input.pixelWidth = target.width;
            input.pixelHeight = target.height;
            host.frame(target.target.Get(), input);
        };
        const auto away = [&](int frames = 1) {
            for (int i = 0; i < frames; ++i) frame({});
        };
        const auto at = [&](float x, float y, bool down) {
            rpsui::imgui_host::PanelInput input{};
            input.pointerValid = true;
            input.pointerX = x;
            input.pointerY = y;
            input.primaryDown = down;
            frame(input);
        };
        const auto click = [&](float x, float y) {
            at(x, y, false);
            at(x, y, true);
            at(x, y, false);
            away(3);
        };
        const auto shot = [&](const char* name) {
            away(2);
            char file[96]{};
            std::snprintf(file, sizeof(file), "%02d-%s.png", ++index, name);
            writePng(target.texture.Get(), target.width, target.height, folder / file);
        };
        const auto open = [&] {
            game.open();
            away(3);
        };
        // Where things are at the default theme.
        constexpr float kFooterY = 768.0f;
        constexpr float kRecommendedX = 160.0f;
        constexpr float kNextX = 1170.0f;
        constexpr float kCloseX = 1202.0f;
        constexpr float kCloseY = 40.0f;
        constexpr std::array kCardY{ 210.0f, 288.0f, 366.0f, 444.0f };

        // Index controllers on SteamVR's default binding: every page, then save.
        open();
        shot("controllers");
        click(640.0f, kCardY[0]);
        shot("controllers-index");
        click(kNextX, kFooterY);
        for (int page = 0; page < 10 && game.panel.pageName() != "Review"; ++page) {
            char label[32]{};
            std::snprintf(label, sizeof(label), "topic-%02d", page + 1);
            shot(label);
            click(kNextX, kFooterY);
        }
        shot("review");
        click(kNextX, kFooterY);
        shot("finished");
        // Vive wands, recommended settings, SteamVR refusing the binding.
        game.steamVr.reported = 1;
        game.steamVr.applySucceeds = false;
        open();
        click(640.0f, kCardY[1]);
        shot("controllers-vive");
        click(kRecommendedX, kFooterY);
        shot("recommended-binding-failed");
        game.steamVr.applySucceeds = true;
        // Touch: nothing to install.
        game.steamVr.reported = 2;
        open();
        click(640.0f, kCardY[2]);
        shot("controllers-touch");
        click(640.0f, kCardY[3]);
        shot("controllers-other");
        // OpenComposite with Index chosen.
        game.steamVr.openComposite = true;
        game.steamVr.index = Binding::Default;
        open();
        click(640.0f, kCardY[0]);
        shot("controllers-opencomposite");
        game.steamVr.openComposite = false;
        // The player's own binding.
        game.steamVr.reported = 0;
        game.steamVr.index = Binding::PlayersOwn;
        open();
        click(640.0f, kCardY[0]);
        shot("controllers-players-own");
        // Closing at once with Index detected: ROCK still installs its binding.
        game.steamVr.index = Binding::Default;
        open();
        click(kCloseX, kCloseY);
        shot("closed-installs-binding");
        return 0;
    }
}
int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, PWSTR, int show)
{
    ImGui_ImplWin32_EnableDpiAwareness();
    (void)CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);

    if (const auto folder = argument(L"--capture"); !folder.empty()) {
        if (FAILED(D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, nullptr, 0, D3D11_SDK_VERSION, &g_device,
                nullptr, &g_context)) &&
            FAILED(D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_WARP, nullptr, 0, nullptr, 0, D3D11_SDK_VERSION, &g_device,
                nullptr, &g_context)))
            return 1;
        // A sandbox of its own, fresh each run, unless --settings names one.
        const bool ownSandbox = argument(L"--settings").empty();
        preview::startSettings(ownSandbox ? fs::absolute(folder) / "settings" : sandboxFolder());
        if (ownSandbox) {
            std::string error;
            if (!preview::resetSettings(error)) return 1;
        }
        Faces faces;
        faces.load();
        int result = 1;
        try {
            result = capture(fs::absolute(folder), faces);
        } catch (const std::exception& error) {
            OutputDebugStringA(error.what());
        }
        CoUninitialize();
        return result;
    }

    WNDCLASSW windowClass{};
    windowClass.lpfnWndProc = windowProc;
    windowClass.hInstance = instance;
    windowClass.lpszClassName = L"ROCKSetupWizardPreview";
    windowClass.hCursor = LoadCursor(nullptr, IDC_ARROW);
    if (!RegisterClassW(&windowClass)) return 1;
    const auto hwnd = CreateWindowW(windowClass.lpszClassName, L"ROCK Setup Wizard Preview", WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, CW_USEDEFAULT, 1840, 1000, nullptr, nullptr, instance, nullptr);
    if (!hwnd) return 1;

    DXGI_SWAP_CHAIN_DESC swapDesc{};
    swapDesc.BufferCount = 2;
    swapDesc.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    swapDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    swapDesc.OutputWindow = hwnd;
    swapDesc.SampleDesc.Count = 1;
    swapDesc.Windowed = TRUE;
    swapDesc.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
    D3D_FEATURE_LEVEL level{};
    HRESULT created = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, nullptr, 0,
        D3D11_SDK_VERSION, &swapDesc, &g_swap, &g_device, &level, &g_context);
    if (FAILED(created))
        created = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_WARP, nullptr, 0, nullptr, 0, D3D11_SDK_VERSION,
            &swapDesc, &g_swap, &g_device, &level, &g_context);
    if (FAILED(created) || !createBackBuffer()) return 1;

    // ROCK's settings store over the sandbox.
    const auto sandbox = sandboxFolder();
    preview::startSettings(sandbox);
    const auto gameSettings = gameModsConfig() / "ROCK" / "ROCK.toml";

    Faces faces;
    faces.load();

    // The preview's own context: its window, controls and the panel's image.
    const float scale = ImGui_ImplWin32_GetDpiScaleForHwnd(hwnd);
    const auto theme = harnessTheme(scale);
    g_harness = ImGui::CreateContext();
    ImGui::SetCurrentContext(g_harness);
    rpsui::imgui_host::configureContext();
    rpsui::imgui_host::applyTheme(theme);
    const auto harnessFonts = rpsui::imgui_host::addThemeFonts(theme, faces.set());
    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX11_Init(g_device.Get(), g_context.Get());

    GameSide game;
    auto& panel = game.panel;
    auto& steamVr = game.steamVr;
    auto& lastFinish = game.lastFinish;
    std::optional<rpsui::imgui_host::LocalPanelHost> host;
    try {
        host.emplace(g_device.Get(), g_context.Get(), registration(panel), faces.set());
    } catch (...) {
        return 1;
    }
    ImGui::SetCurrentContext(g_harness);

    PanelTarget panelTarget;
    std::string sandboxMessage;
    bool followLog = true;
    const auto openWizard = [&] { game.open(); };
    const auto closeWizard = [&] { game.close(); };
    openWizard();

    ShowWindow(hwnd, show);
    UpdateWindow(hwnd);
    bool done = false;
    while (!done) {
        ImGui::SetCurrentContext(g_harness);
        MSG message{};
        while (PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE)) {
            TranslateMessage(&message);
            DispatchMessageW(&message);
            if (message.message == WM_QUIT) done = true;
        }
        if (done) break;
        if (IsIconic(hwnd)) {
            WaitMessage();
            continue;
        }
        if (g_resizeWidth && g_resizeHeight) {
            g_backBuffer.Reset();
            if (FAILED(g_swap->ResizeBuffers(0, g_resizeWidth, g_resizeHeight, DXGI_FORMAT_UNKNOWN, 0)) || !createBackBuffer())
                break;
            g_resizeWidth = g_resizeHeight = 0;
        }
        preview::pumpSettings();

        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();
        auto& io = ImGui::GetIO();
        const bool keysAllowed = !io.WantTextInput;
        bool backPressed = keysAllowed && ImGui::IsKeyPressed(ImGuiKey_Backspace, false);

        rpsui::imgui_host::PanelInput panelInput{};
        panelInput.deltaSeconds = io.DeltaTime;

        ImGui::SetNextWindowPos({ 0.0f, 0.0f });
        ImGui::SetNextWindowSize(io.DisplaySize);
        ImGui::Begin("##preview", nullptr,
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
                ImGuiWindowFlags_NoBringToFrontOnFocus);
        const float sideWidth = (std::min)(560.0f * scale, ImGui::GetContentRegionAvail().x * 0.42f);

        // ---- the panel ----
        ImGui::BeginChild("panel-area", { ImGui::GetContentRegionAvail().x - sideWidth - theme.spacing, 0.0f },
            ImGuiChildFlags_None, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        if (panel.isOpen()) {
            const auto area = ImGui::GetContentRegionAvail();
            const float aspect = wizard::kLayoutWidth / wizard::kLayoutHeight;
            ImVec2 size{ area.x, area.x / aspect };
            if (size.y > area.y) size = { area.y * aspect, area.y };
            const ImVec2 origin{ ImGui::GetCursorScreenPos().x + (area.x - size.x) * 0.5f, ImGui::GetCursorScreenPos().y };
            const auto pixelWidth = static_cast<UINT>(size.x * io.DisplayFramebufferScale.x);
            const auto pixelHeight = static_cast<UINT>(size.y * io.DisplayFramebufferScale.y);
            if (panelTarget.ensure(pixelWidth, pixelHeight)) {
                ImGui::SetCursorScreenPos(origin);
                ImGui::Image(ImTextureRef(reinterpret_cast<ImTextureID>(panelTarget.view.Get())), size);
                ImGui::SetCursorScreenPos(origin);
                // Keeps the preview's own window from taking the clicks meant for the panel.
                ImGui::InvisibleButton("panel-surface", size);
                const ImVec2 mouse = io.MousePos;
                const bool inside = mouse.x >= origin.x && mouse.y >= origin.y && mouse.x < origin.x + size.x &&
                    mouse.y < origin.y + size.y && ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);
                panelInput.pixelWidth = panelTarget.width;
                panelInput.pixelHeight = panelTarget.height;
                panelInput.pointerValid = inside;
                panelInput.pointerX = (mouse.x - origin.x) / size.x * wizard::kLayoutWidth;
                panelInput.pointerY = (mouse.y - origin.y) / size.y * wizard::kLayoutHeight;
                panelInput.primaryDown = inside && io.MouseDown[ImGuiMouseButton_Left];
                // RPS UI turns stick deflection into wheel units; this gives back the mouse's own.
                if (inside && io.MouseWheel != 0.0f)
                    panelInput.scrollAxisY = io.MouseWheel / (rpsui::imgui_host::kWheelUnitsPerSecond * (std::max)(io.DeltaTime, 1.0f / 240.0f));
                if (inside && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) backPressed = true;
            }
        } else {
            ImGui::Dummy({ 0.0f, ImGui::GetContentRegionAvail().y * 0.4f });
            ImGui::TextUnformatted("The setup wizard is closed.");
            if (ImGui::Button("Open the setup wizard")) openWizard();
        }
        ImGui::EndChild();
        ImGui::SameLine();

        // ---- what the game would supply ----
        ImGui::BeginChild("side", { 0.0f, 0.0f }, ImGuiChildFlags_None);
        ImGui::PushFont(harnessFonts[static_cast<std::size_t>(fw::FontRole::Strong)], theme.titleFontSize);
        ImGui::TextUnformatted("ROCK Setup Wizard Preview");
        ImGui::PopFont();
        muted(theme, "The panel on the left is ROCK's setup wizard, drawn by the same code the game runs. "
                     "Point and click it with the mouse; scroll with the wheel. Right click on it, or "
                     "Backspace, is the grip tap that goes Back.");

        if (ImGui::CollapsingHeader("Wizard", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text("%s", panel.isOpen() ? ("Open: " + panel.pageName()).c_str() : "Closed");
            if (ImGui::Button(panel.isOpen() ? "Open again (new session)" : "Open (as from PALM's Config)")) openWizard();
            ImGui::SameLine();
            ImGui::BeginDisabled(!panel.isOpen());
            if (ImGui::Button("Close")) closeWizard();
            ImGui::EndDisabled();
            muted(theme, "Opening reads the sandbox settings and SteamVR's state, as the game does each time the wizard opens.");
        }

        if (ImGui::CollapsingHeader("SteamVR", ImGuiTreeNodeFlags_DefaultOpen)) {
            muted(theme, "What the game would find when the wizard opens. Changes show the next time it opens.");
            ImGui::Checkbox("The game runs through OpenComposite", &steamVr.openComposite);
            ImGui::BeginDisabled(steamVr.openComposite);
            std::array<const char*, kReported.size()> reported{};
            for (std::size_t i = 0; i < kReported.size(); ++i) reported[i] = kReported[i].label;
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.62f);
            ImGui::Combo("Controllers on", &steamVr.reported, reported.data(), static_cast<int>(reported.size()));
            const char* bindings[] = { "SteamVR's default binding", "ROCK's binding", "A binding the player chose" };
            for (auto* binding : { &steamVr.index, &steamVr.vive }) {
                int value = static_cast<int>(*binding);
                ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.62f);
                if (ImGui::Combo(binding == &steamVr.index ? "Index binding" : "Vive wand binding", &value, bindings,
                        IM_ARRAYSIZE(bindings)))
                    *binding = static_cast<Binding>(value);
            }
            ImGui::EndDisabled();
            ImGui::Checkbox("Installing ROCK's binding succeeds", &steamVr.applySucceeds);
            if (!steamVr.applySucceeds) ImGui::InputText("Error", steamVr.applyError.data(), steamVr.applyError.size());
            muted(theme, "Installing here only changes this preview's SteamVR state; SteamVR itself is never touched.");
        }

        if (ImGui::CollapsingHeader("Headset input", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Back (grip tap at the panel)")) backPressed = true;
            ImGui::SameLine();
            muted(theme, "Backspace or a right click on the panel");
        }
        if (ImGui::CollapsingHeader("Settings sandbox", ImGuiTreeNodeFlags_DefaultOpen)) {
            const auto folder = preview::settingsDirectory();
            muted(theme, "ROCK's settings store reads and writes here. The game's settings are never written.");
            ImGui::TextWrapped("%s", folder.string().c_str());
            if (ImGui::Button("Reset to ROCK's defaults")) {
                std::string error;
                sandboxMessage = preview::resetSettings(error) ? "Reset to ROCK's defaults." : error;
                if (panel.isOpen()) openWizard();
            }
            ImGui::SameLine();
            const bool haveGame = fs::exists(gameSettings);
            ImGui::BeginDisabled(!haveGame);
            if (ImGui::Button("Copy the game's ROCK.toml")) {
                std::string error;
                sandboxMessage = preview::copySettings(gameSettings, error) ? "Copied the game's settings." : error;
                if (panel.isOpen()) openWizard();
            }
            ImGui::EndDisabled();
            if (ImGui::Button("Open ROCK.toml")) openInShell(folder / "ROCK.toml");
            ImGui::SameLine();
            if (ImGui::Button("Open the folder")) openInShell(folder);
            if (!sandboxMessage.empty()) muted(theme, sandboxMessage.c_str());
        }

        if (ImGui::CollapsingHeader("Last finish", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (!lastFinish) muted(theme, "Not finished in this run.");
            else {
                if (lastFinish->written.empty() && lastFinish->errors.empty()) muted(theme, "No setting written.");
                for (const auto& write : lastFinish->written)
                    ImGui::BulletText("[%s] %s = %s", write.section.c_str(), write.key.c_str(), write.value.c_str());
                if (lastFinish->bindingInstalled) ImGui::BulletText("ROCK's SteamVR binding installed");
                ImGui::PushStyleColor(ImGuiCol_Text, rpsui::imgui_host::color(theme.invalid));
                for (const auto& error : lastFinish->errors) ImGui::BulletText("%s", error.c_str());
                if (!lastFinish->bindingError.empty()) ImGui::BulletText("Binding: %s", lastFinish->bindingError.c_str());
                ImGui::PopStyleColor();
            }
        }
        if (ImGui::CollapsingHeader("ROCK log", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Follow", &followLog);
            ImGui::SameLine();
            if (ImGui::Button("Clear")) preview::clearLog();
            ImGui::BeginChild("log", { 0.0f, 220.0f * scale }, ImGuiChildFlags_Borders, ImGuiWindowFlags_HorizontalScrollbar);
            ImGui::PushFont(harnessFonts[static_cast<std::size_t>(fw::FontRole::Mono)], theme.smallFontSize);
            for (const auto& line : preview::logLines()) ImGui::TextUnformatted(line.c_str());
            ImGui::PopFont();
            if (followLog && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 4.0f) ImGui::SetScrollHereY(1.0f);
            ImGui::EndChild();
        }
        ImGui::EndChild();
        ImGui::End();

        game.service();
        if (panel.isOpen() && panelTarget.target) {
            const float clear[4]{ 0.0f, 0.0f, 0.0f, 0.0f };
            g_context->ClearRenderTargetView(panelTarget.target.Get(), clear);
            host->frame(panelTarget.target.Get(), panelInput, fw::PhysicalHand::Right, backPressed);
            ImGui::SetCurrentContext(g_harness);
        }

        ImGui::Render();
        auto* backBuffer = g_backBuffer.Get();
        g_context->OMSetRenderTargets(1, &backBuffer, nullptr);
        const auto& background = theme.background;
        const float clear[4]{ background.r * 0.6f, background.g * 0.6f, background.b * 0.6f, 1.0f };
        g_context->ClearRenderTargetView(backBuffer, clear);
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
        if (FAILED(g_swap->Present(1, 0))) break;
    }

    host.reset();
    ImGui::SetCurrentContext(g_harness);
    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext(g_harness);
    g_harness = nullptr;
    DestroyWindow(hwnd);
    UnregisterClassW(windowClass.lpszClassName, instance);
    CoUninitialize();
    return 0;
}
