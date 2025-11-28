#pragma once

// ============================================
//            File: Window.h
// ============================================
// Interface for a basic windowing system.
//
// Summary:
// ============================================
//
// interface:
// --------------------------------------------
// IWindow
//      -> Interface class defining basic window operations and callbacks.
// --------------------------------------------
//
// public:
// --------------------------------------------
// virtual bool init(int width, int height, const std::string& title)
//      -> Initializes the window with the specified width, height, and title.
// virtual bool isRunning()
//      -> Checks if the window is currently running.
// virtual bool shouldClose()
//      -> Checks if the window should close.
// virtual void pollEvents()
//      -> Polls for window events.
// virtual void swapBuffers()
//      -> Swaps the front and back buffers of the window.
// virtual void* getNativeWin()
//      -> Returns the native window pointer.
// virtual void setNativeWin(void* window)
//      -> Sets the native window pointer.
// virtual void onKey(int key, int scancode, int action, int mods)
//      -> Handles key press events.
// virtual void onScroll(double delta)
//      -> Handles mouse scroll events.
// virtual void onResize(int width, int height)
//      -> Handles window resize events.
// virtual void onCursorPos(double xpos, double ypos)
//      -> Handles mouse cursor position events.
// virtual void onClose()
//      -> Handles window close events.
// virtual ~IWindow()
//      -> Virtual destructor for the IWindow interface.
// virtual int getWidth()
//      -> Returns the width of the window.
// virtual int getHeight()
//      -> Returns the height of the window.
// virtual const std::string& getHeader()
//      -> Returns the header string of the window.
// --------------------------------------------
// 
// ============================================
//			  GitHub: saltyjoss
// ============================================

#include "EngineCore.h"

#include <imgui.h>
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

//Basic window interface
namespace window {
    class ENGINE_API IWindow {
    public:
        virtual bool init(int width, int height, const std::string& title) = 0;

        // Core windowing
        virtual bool isRunning() const = 0;
        virtual bool shouldClose() const = 0;
        virtual void pollEvents() = 0;
        virtual void swapBuffers() = 0;

        // Native window access
        virtual void* getNativeWin() = 0;
        virtual void setNativeWin(void* window) = 0;

        // Callbacks
        virtual void onKey(int key, int scancode, int action, int mods) = 0;
        virtual void onScroll(double delta) = 0;
        virtual void onResize(int width, int height) = 0;
		virtual void onCursorPos(double xpos, double ypos) = 0;
        virtual void onClose() = 0;

        virtual ~IWindow() = default;

        virtual int getWidth() const = 0;
        virtual int getHeight() const = 0;
        virtual const std::string& getHeader() const = 0;
    };
}