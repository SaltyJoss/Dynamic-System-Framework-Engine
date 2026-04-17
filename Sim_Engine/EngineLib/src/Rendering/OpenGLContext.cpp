#include "pch.h"
// File:   OpenGLContext.cpp
// GitHub: SaltyJoss
#ifdef __gl_h_
#undef __gl_h_
#endif
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "Rendering/OpenGLContext.h"
#include <Scene/SimulationManager.h>

#include "EngineLib/LogMacros.h"

namespace render {
	// GLFW Callback for key events
	static void onKey_Callback(GLFWwindow* win, int  key, int scancode, int action, int mods) {
		auto currentWindow = static_cast<window::IWindow*>(glfwGetWindowUserPointer(win));
		currentWindow->onKey(key, scancode, action, mods);
	}

	// GLFW Callback for cursor position events
	static void CursorPos_Callback(GLFWwindow* win, double xpos, double ypos) {
		auto currentWindow = static_cast<window::IWindow*>(glfwGetWindowUserPointer(win));
		currentWindow->onCursorPos(xpos, ypos);
	}

	// GLFW Callback for scroll events
	static void onScroll_Callback(GLFWwindow* win, double /*xoffset*/, double yoffset) {
		auto currentWindow = static_cast<window::IWindow*>(glfwGetWindowUserPointer(win));
		currentWindow->onScroll(yoffset);
	}

	// GLFW Callback for window resize events
	static void onResize_Callback(GLFWwindow* win, int width, int height) {
		auto currentWindow = static_cast<window::IWindow*>(glfwGetWindowUserPointer(win));
		currentWindow->onResize(width, height);
	}

	// GLFW Callback for window close events
	static void onClose_Callback(GLFWwindow* win) {
		window::IWindow* currentWindow = static_cast<window::IWindow*>(glfwGetWindowUserPointer(win));
		currentWindow->onClose();
	}

	// Initialize OpenGL context and create GLFW window
	bool render::OpenGLContext::init(window::IWindow* window) {
		__super::init(window);

		if (!window->getWidth() || !window->getHeight()) {
			LOG_ERROR("Window dimensions not set!");
			return false;
		}

		if (!glfwInit()) { 
			LOG_ERROR("Failed to initialize GLFW -> ", glfwGetError(NULL));
			return false; 
		}

		// Set GLFW window hints for OpenGL version and profile
		auto glWindow = glfwCreateWindow(window->getWidth(), window->getHeight(), window->getHeader().c_str(), nullptr, nullptr);
		glfwSetInputMode(glWindow, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
		glfwSetInputMode(glWindow, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);

		// Check if window creation succeeded
		if (!glWindow) {
			LOG_ERROR("Failed to create GLFW window -> ", glfwGetError(NULL));
			glfwTerminate();
			return false;
		}

		window->setNativeWin(glWindow);
		_glfwWindow = glWindow;

		// Set up OpenGL context and callbacks
		glfwSwapInterval(1);
		glfwSetWindowUserPointer(glWindow, window); // Set user pointer to access window instance in callbacks
		glfwSetWindowSizeLimits(glWindow, 800, 450, GLFW_DONT_CARE, GLFW_DONT_CARE); // Minimum 16:9 at 800x450, no maximum
		glfwSetWindowAspectRatio(glWindow, 16, 9); // Enforce 16:9 aspect ratio
		glfwSetCursorPosCallback(glWindow, CursorPos_Callback);
		glfwSetKeyCallback(glWindow, onKey_Callback);
		glfwSetScrollCallback(glWindow, onScroll_Callback);
		glfwSetWindowSizeCallback(glWindow, onResize_Callback);
		glfwSetWindowCloseCallback(glWindow, onClose_Callback);
		glfwMakeContextCurrent(glWindow);

		// Load OpenGL function pointers using GLAD
		if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) { 
			LOG_ERROR("Failed to initialise GLAD");
			return false; 
		}

		glEnable(GL_DEPTH_TEST);
		return true;
	}

	// Set viewport and clear buffers before rendering
	void render::OpenGLContext::preRender() {
		glViewport(0, 0, _window->getWidth(), _window->getHeight());
		glClearColor(0.33f, 0.33f, 0.33f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}

	// Swap buffers after rendering
	void render::OpenGLContext::postRender() {
		glfwSwapBuffers((GLFWwindow*)_window->getNativeWin());
	}

	// Clean up GLFW resources and terminate context
	void render::OpenGLContext::end() {
		glfwDestroyWindow((GLFWwindow*)_window->getNativeWin());
		glfwTerminate();
	}
}
