// DSFE_GUI OpenGLContext.h
#pragma once

#include "RenderBase.h"
#include "ui/Styles.h"
#include "Platform/Logger.h"

namespace render {
	class OpenGLContext : public RenderContext {
	public:
		bool init(window::IWindow* window) override;
		void preRender() override;
		void postRender() override;
		void end() override;

		GLFWwindow* getGLFWWindow() const { return _glfwWindow; }

	private:
		GLFWwindow* _glfwWindow = nullptr;
	};
} // namespace render