#pragma once

// =============================================
//            File: GUIContext.h
// =============================================
// Class representing a GUI rendering context.
//
// Summary:
// =============================================
//
// public:
// --------------------------------------------
// GUIContext()
//      -> Constructor that initializes the GUIContext.
// bool init(window::IWindow* win) override
//      -> Initializes the GUI context with the given window.
// void preRender() override
//      -> Prepares the GUI context for rendering.
// void postRender() override
//      -> Finalizes the GUI context after rendering.
// void end() override
//      -> Cleans up the GUI context resources.
// --------------------------------------------
//
// private:
// --------------------------------------------
// std::unique_ptr<gui::Styles> _style
//      -> Unique pointer to the GUI styles manager.
// --------------------------------------------
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
	class ENGINE_API GUIContext : public RenderContext {
	public:
		GUIContext() {
			
		}

		bool init(window::IWindow* win) override;
		void preRender() override;
		void postRender() override;
		void end() override;

		void setMenuCallback(std::function<void()> callback) { _menuCallback = std::move(callback); }

	private:
		std::unique_ptr<gui::Styles> _style;
		std::function<void()> _menuCallback;
	};
}