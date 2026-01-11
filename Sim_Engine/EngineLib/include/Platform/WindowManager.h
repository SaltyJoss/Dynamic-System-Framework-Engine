#pragma once

//=============================================
//            File: WindowManager.h
//=============================================
// GLFW-based implementation of the IWindow interface.
// 
// Summary:
// ============================================
// 
// public:
// --------------------------------------------
// GLWindow() 
//      -> Constructor that initializes the GLWindow object.
// ~GLWindow() 
//      -> Destructor that cleans up resources and ends rendering and GUI contexts.
// render() 
//      -> Renders the scene view, control panel, and debug panel.
// init(int width, int height, const std::string& header) 
//      -> Initializes the GLWindow with the specified width, height, and header.
// onResize(int width, int height) 
//      -> Handles window resize events and updates the scene view accordingly.
// shouldClose() 
//      -> Checks if the window should close.
// pollEvents() 
//      -> Polls for window events.
// swapBuffers() 
//      -> Swaps the front and back buffers of the window.
// getNativeWin() 
//      -> Returns the native GLFW window pointer.
// setNativeWin(void* window) 
//      -> Sets the native GLFW window pointer.
// getWidth() 
//      -> Returns the width of the window.
// getHeight() 
//      -> Returns the height of the window.
// getHeader() 
//      -> Returns the header string of the window.
// setMouseCaptured(bool captured)
//      -> Sets whether the mouse is captured (disabled) or not.
// isMouseCaptured()
// 	    -> Checks if the mouse is currently captured.
// onKey(int key, int scancode, int action, int mods)
//      -> Handles key press events.
// onScroll(double delta)
//      -> Handles mouse scroll events.
// onResize(int width, int height)
//      -> Handles window resize events.
// onCursorPos(double xpos, double ypos)
//      -> Handles mouse cursor position events.
// onClose()
//      -> Handles window close events.
// update()
//      -> Updates the window state, handles input, and applies camera movement and gravity.
// --------------------------------------------
// 
// Internal State Variables:
// --------------------------------------------
// struct UIState
//      -> Internal structure to hold the state of various UI panels.
// bool _isRunning
//      -> Indicates whether the window is currently running.
// GLFWwindow* _window
//      -> Pointer to the native GLFW window.
// std::unique_ptr<render::GUIContext> _GUICntx
//  	-> Unique pointer to the GUI context for rendering GUI scene.
// std::unique_ptr<render::OpenGLContext> _renderCntx
//      -> Unique pointer to the OpenGL rendering context.
// std::unique_ptr<gui::simManager> _sim
//      -> Unique pointer to the scene view for rendering 3D scenes.
// std::unique_ptr<gui::ControlPanel> _controlPanel
//      -> Unique pointer to the control panel for user interactions.
// std::unique_ptr<gui::DebugPanel> _debugPanel
// 	    -> Unique pointer to the debug panel for displaying debug information.
// bool _isHovered
//      -> Indicates whether the window is currently hovered by the mouse.
// bool _mouseCaptured
//      -> Indicates whether the mouse is currently captured (disabled).
// int _width
//      -> Width of the window.
// int _height
//      -> Height of the window.
// std::string* _header
// 	    -> Pointer to the header string of the window.
// --------------------------------------------
// 
// Built upon code from:
// ============================================
//	 GitHub: jayanam/jgl_demos/JGL_MeshLoader
// ============================================
// 
// ============================================
//			  GitHub: saltyjoss
// ============================================

#include "EngineCore.h"

#include "Platform/Window.h"
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

// Forward Declarations
struct GLFWwindow;

namespace render { 
    class GUIContext; 
    class OpenGLContext; 
}
namespace gui { 
    class simManager; 
    class ControlPanel; 
    class DebugPanel;
	class CommandScriptEditor;
}

namespace window {
    class ENGINE_API GLWindow : public IWindow {
    public:
        GLWindow();
        ~GLWindow();

        bool init(int width, int height, const std::string& title) override;

        // IWindow interface
        bool isRunning() const override;
        bool shouldClose() const override;
        void pollEvents() override;
        void swapBuffers() override;

        void* getNativeWin() override;
        void setNativeWin(void* window) override;

        int getWidth() const override;
        int getHeight() const override;
        const std::string& getHeader() const override;

        void setMouseCaptured(bool captured);
        bool isMouseCaptured() const { return _mouseCaptured; }
        void onKey(int key, int scancode, int action, int mods) override;
        void onScroll(double delta) override;
        void onResize(int width, int height) override;
        void onCursorPos(double xpos, double ypos) override;
        void onClose() override;

        void update();
        void render();

    private:

        struct UIState {
			bool sceneViewOpen = true;
			bool controlPanelOpen = true;
			bool debugPanelOpen = true;
        };

        bool _isRunning = true;
        GLFWwindow* _window = nullptr;

        std::unique_ptr<render::GUIContext> _GUICntx;
        std::unique_ptr<render::OpenGLContext> _renderCntx;

        std::unique_ptr<gui::simManager> _sim;
        std::unique_ptr<gui::ControlPanel> _controlPanel;
        std::unique_ptr<gui::DebugPanel> _debugPanel;
		std::unique_ptr<gui::CommandScriptEditor> _cmdEditor;

        bool _isHovered = false;
        bool _mouseCaptured = false;
        

        int _width = 0;
        int _height = 0;
        std::string *_header;
    };
}