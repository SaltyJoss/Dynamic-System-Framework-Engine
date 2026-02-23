#pragma once
// File:   WindowManager.h
// GitHub: SaltyJoss
#include "EngineCore.h"
#include "Platform/Window.h"
#include "Platform/Logger.h"

// Forward Declarations
struct ENGINE_API GLFWwindow;
namespace render { 
    class ENGINE_API GUIContext;
    class ENGINE_API OpenGLContext;
}
namespace gui { 
    class ENGINE_API SimManager;
    class ENGINE_API ControlPanel;
    class ENGINE_API DebugPanel;
	class ENGINE_API CommandScriptEditor;
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

		// Input handling
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
		// UI State
        struct UIState {
			bool sceneViewOpen = true;
			bool controlPanelOpen = true;
			bool debugPanelOpen = true;
        };

        bool _isRunning = true;
        GLFWwindow* _window = nullptr;

        std::unique_ptr<render::GUIContext> _GUICntx;
        std::unique_ptr<render::OpenGLContext> _renderCntx;
        std::unique_ptr<gui::SimManager> _sim;
        std::unique_ptr<gui::ControlPanel> _controlPanel;
        std::unique_ptr<gui::DebugPanel> _debugPanel;
		std::unique_ptr<gui::CommandScriptEditor> _cmdEditor;

        bool _isHovered = false;
        bool _mouseCaptured = true;
        
		// Window properties
        int _width = 0;
        int _height = 0;
        std::string *_header;
    };
} // namespace window