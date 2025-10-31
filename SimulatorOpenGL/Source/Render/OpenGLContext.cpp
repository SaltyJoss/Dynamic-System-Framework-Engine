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

		window::IWindow* localWindow = window; // local copy for logging/debug

		fprintf(stderr, "[INSIDE OpenGLContext::Init()]Width: %d, Height: %d, Header: %s\n", window->_width, window->_height, window->_header.c_str());

		if (!window->_width || !window->_height) {
			fprintf(stderr, "[ERROR / OpenGL] OpenGLInit: Window dimensions not set!\n");
			return false;
		}

		if (!glfwInit()) { 
			fprintf(stderr, "[ERROR / OpenGL] OpenGLInit: Failed to initialize GLFW\n");
			return false; 
		}

		auto glWindow = glfwCreateWindow(window->_width, window->_height, window->_header.c_str(), nullptr, nullptr);
		localWindow->setNativeWin(glWindow);

		if (!glWindow) { 
			fprintf(stderr, "[ERROR / OpenGL] OpenGLInit: Failed to create GLFW window\n");
			glfwTerminate();
			return false;
		}

		glfwSetWindowUserPointer(glWindow, window);
		glfwSetKeyCallback(glWindow, onKey_Callback);
		glfwSetScrollCallback(glWindow, onScroll_Callback);
		glfwSetWindowSizeCallback(glWindow, onResize_Callback);
		glfwSetWindowCloseCallback(glWindow, onClose_Callback);
		glfwMakeContextCurrent(glWindow);

		GLenum err = glewInit();
		if (err != GLEW_OK) { 
			fprintf(stderr, "[ERROR / OpenGL] OpenGLInit: GLEW failed: %s\n", glewGetErrorString(err)); 
			return false; 
		}

		if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) { std::cerr << "Failed to initialize GLAD" << std::endl; return false; }

		glEnable(GL_DEPTH_TEST);

		return true;
	}

	void render::OpenGLContext::preRender() {
		glViewport(0, 0, _window->_width, _window->_height);
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