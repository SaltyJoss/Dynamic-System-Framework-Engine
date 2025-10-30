#include "ch.h"
#include "OpenGLContext.h"

namespace render {
	static void onKey_Callback(GLFWwindow* win, int  key, int scancode, int action, int mods) {
		auto currentWindow = static_cast<window::IWindow*>(glfwGetWindowUserPointer(win));
		currentWindow->onKey(key, scancode, action, mods);
	}

	static void onScroll_Callback(GLFWwindow* win, double xoffset, double yoffset) {
		auto currentWindow = static_cast<window::IWindow*>(glfwGetWindowUserPointer(win));
		currentWindow->onScroll(yoffset);
	}

	static void onResize_Callback(GLFWwindow* win, int width, int height) {
		auto currentWindow = static_cast<window::IWindow*>(glfwGetWindowUserPointer(win));
		currentWindow->onResize(width, height);
	}

	static void onClose_Callback(GLFWwindow* win) {
		window::IWindow* currentWindow = static_cast<window::IWindow*>(glfwGetWindowUserPointer(win));
		currentWindow->onClose();
	}

	bool render::OpenGLContext::init(window::IWindow* window) {
		__super::init(window);

		if (!glfwInit()) { std::cerr << "[ERROR]: " << "Failed to initialize GLFW" << std::endl; return false; }

		auto glWindow = glfwCreateWindow(window->width, window->height, window->header.c_str(), nullptr, nullptr);
		window->setNativeWin(glWindow);

		if (!glWindow) { std::cerr << "[ERROR]: " << "Failed to create GLFW window" << std::endl; return false; }

		glfwSetWindowUserPointer(glWindow, window);
		glfwSetKeyCallback(glWindow, onKey_Callback);
		glfwSetScrollCallback(glWindow, onScroll_Callback);
		glfwSetWindowSizeCallback(glWindow, onResize_Callback);
		glfwSetWindowCloseCallback(glWindow, onClose_Callback);
		glfwMakeContextCurrent(glWindow);

		GLenum err = glewInit();
		if (GLEW_OK != err) { std::cerr << "[ERROR]: " << stderr << "\n" << glewGetErrorString(err) << std::endl; return false; }

		glEnable(GL_DEPTH_TEST);

		return true;
	}

	void render::OpenGLContext::preRender() {
		glViewport(0, 0, _window->width, _window->height);
		glClearColor(0.33f, 0.33f, 0.33f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}

	void render::OpenGLContext::postRender() {
		glfwPollEvents();
		glfwSwapBuffers((GLFWwindow*)_window->getNativeWin());
	}

	void render::OpenGLContext::end() {
		glfwDestroyWindow((GLFWwindow*)_window->getNativeWin());
		glfwTerminate();
	}
}