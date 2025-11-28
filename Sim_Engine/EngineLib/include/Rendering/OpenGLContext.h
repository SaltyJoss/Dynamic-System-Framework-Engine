#pragma once

// =============================================
//            File: OpenGLContext.h
// =============================================
// Class representing an OpenGL rendering context.
//
// Summary:
// =============================================
//
// public:
// --------------------------------------------
// OpenGLContext()
//      -> Constructor for the OpenGLContext class.
// bool init(window::IWindow* window) override
//      -> Initializes the OpenGL context with the given window.
// void preRender() override
//      -> Prepares the context for rendering.
// void postRender() override
//      -> Finalizes the rendering process.
// void end() override
//      -> Cleans up the OpenGL context.
// GLFWwindow* getGLFWWindow() const
//      -> Returns the underlying GLFW window pointer.
// --------------------------------------------
//
// private:
// --------------------------------------------
// GLFWwindow* _glfwWindow
//      -> Pointer to the GLFW window associated with the OpenGL context.
// --------------------------------------------
//
// Built upon code from:
// ============================================
//	 GitHub: jayanam/jgl_demos/JGL_MeshLoader
// ============================================
// 
// ============================================
//			  GitHub: SaltyJoss
// ============================================

#include "EngineCore.h"

#include "RenderBase.h"
#include "Scene/Styles.h"
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

namespace render {
	class ENGINE_API OpenGLContext : public RenderContext {
	public:
		bool init(window::IWindow* window) override;
		void preRender() override;
		void postRender() override;
		void end() override;

		GLFWwindow* getGLFWWindow() const { return _glfwWindow; }

	private:
		GLFWwindow* _glfwWindow = nullptr;
	};
}