/*
 * Public PrismaUI base API.
 *
 * Consumers may copy this header into their own project. The V1-V4 layouts are
 * intentionally identical to the flat Fallout 4 provider.
 */
#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#endif

#ifndef NOMINMAX
#    define NOMINMAX
#endif

#include <Windows.h>

#include <cstdint>

using PrismaView = std::uint64_t;

namespace PRISMA_UI_API
{
    inline constexpr auto PrismaUIPluginName = "PrismaUI_F4";

    enum class InterfaceVersion : std::uint8_t
    {
        V1 = 0,
        V2 = 1,
        V3 = 2,
        V4 = 3
    };

    using OnDomReadyCallback = void (*)(PrismaView view);
    using JSCallback = void (*)(const char* result);
    using JSListenerCallback = void (*)(const char* argument);

    enum class ConsoleMessageLevel : std::uint8_t
    {
        Log = 0,
        Warning,
        Error,
        Debug,
        Info
    };

    using ConsoleMessageCallback =
        void (*)(PrismaView view, ConsoleMessageLevel level, const char* message);
    using ViewEnumCallback =
        void (*)(PrismaView view, const char* htmlPath, void* userData);

    class IVPrismaUI1
    {
    protected:
        ~IVPrismaUI1() = default;

    public:
        virtual PrismaView CreateView(
            const char* htmlPath,
            OnDomReadyCallback onDomReadyCallback = nullptr) noexcept = 0;
        virtual void Invoke(
            PrismaView view,
            const char* script,
            JSCallback callback = nullptr) noexcept = 0;
        virtual void InteropCall(
            PrismaView view,
            const char* functionName,
            const char* argument) noexcept = 0;
        virtual void RegisterJSListener(
            PrismaView view,
            const char* functionName,
            JSListenerCallback callback) noexcept = 0;
        virtual bool HasFocus(PrismaView view) noexcept = 0;
        virtual bool Focus(
            PrismaView view,
            bool pauseGame = false,
            bool disableFocusMenu = false) noexcept = 0;
        virtual void Unfocus(PrismaView view) noexcept = 0;
        virtual void Show(PrismaView view) noexcept = 0;
        virtual void Hide(PrismaView view) noexcept = 0;
        virtual bool IsHidden(PrismaView view) noexcept = 0;
        virtual int GetScrollingPixelSize(PrismaView view) noexcept = 0;
        virtual void SetScrollingPixelSize(PrismaView view, int pixelSize) noexcept = 0;
        virtual bool IsValid(PrismaView view) noexcept = 0;
        virtual void Destroy(PrismaView view) noexcept = 0;
        virtual void SetOrder(PrismaView view, int order) noexcept = 0;
        virtual int GetOrder(PrismaView view) noexcept = 0;
        virtual void CreateInspectorView(PrismaView view) noexcept = 0;
        virtual void SetInspectorVisibility(PrismaView view, bool visible) noexcept = 0;
        virtual bool IsInspectorVisible(PrismaView view) noexcept = 0;
        virtual void SetInspectorBounds(
            PrismaView view,
            float topLeftX,
            float topLeftY,
            unsigned int width,
            unsigned int height) noexcept = 0;
        virtual bool HasAnyActiveFocus() noexcept = 0;
    };

    class IVPrismaUI2 : public IVPrismaUI1
    {
    protected:
        ~IVPrismaUI2() = default;

    public:
        virtual void RegisterConsoleCallback(
            PrismaView view,
            ConsoleMessageCallback callback) noexcept = 0;
    };

    class IVPrismaUI3 : public IVPrismaUI2
    {
    protected:
        ~IVPrismaUI3() = default;

    public:
        virtual void RegisterTranslations(
            PrismaView view,
            const char* pluginName) noexcept = 0;
    };

    class IVPrismaUI4 : public IVPrismaUI3
    {
    protected:
        ~IVPrismaUI4() = default;

    public:
        virtual void BindUIEvent(
            PrismaView view,
            const char* functionName,
            JSListenerCallback callback) noexcept = 0;
        virtual void EnumerateViews(
            ViewEnumCallback callback,
            void* userData) noexcept = 0;
    };

    template <class Interface>
    struct InterfaceVersionMap;

    template <>
    struct InterfaceVersionMap<IVPrismaUI1>
    {
        static constexpr auto version = InterfaceVersion::V1;
    };

    template <>
    struct InterfaceVersionMap<IVPrismaUI2>
    {
        static constexpr auto version = InterfaceVersion::V2;
    };

    template <>
    struct InterfaceVersionMap<IVPrismaUI3>
    {
        static constexpr auto version = InterfaceVersion::V3;
    };

    template <>
    struct InterfaceVersionMap<IVPrismaUI4>
    {
        static constexpr auto version = InterfaceVersion::V4;
    };

    using RequestPluginAPIFunc = void* (*)(InterfaceVersion version);

    [[nodiscard]] inline void* RequestPluginAPI(
        InterfaceVersion version = InterfaceVersion::V1) noexcept
    {
        const auto module = GetModuleHandleW(L"PrismaUI_F4.dll");
        if (!module) {
            return nullptr;
        }

        const auto request = reinterpret_cast<RequestPluginAPIFunc>(
            GetProcAddress(module, "RequestPluginAPI"));
        return request ? request(version) : nullptr;
    }

    template <class Interface>
    [[nodiscard]] inline Interface* RequestPluginAPI() noexcept
    {
        return static_cast<Interface*>(
            RequestPluginAPI(InterfaceVersionMap<Interface>::version));
    }
}
