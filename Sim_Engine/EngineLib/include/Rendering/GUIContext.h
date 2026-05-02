#pragma once
// File:    GUIContext.h
// GitHub:  SaltyJoss
#include "EngineCore.h"
#include "RenderBase.h"
#include "ui/Styles.h"
#include "Platform/Logger.h"

namespace render {
	class DSFE_API GUIContext : public RenderContext {
	public:
		GUIContext() {}

		bool init(window::IWindow* win) override;
		void preRender() override;
		void postRender() override;
		void end() override;

		void setMenuCallback(std::function<void()> callback) { _menuCallback = std::move(callback); }

	private:
		std::unique_ptr<gui::Styles> _style;
		std::function<void()> _menuCallback;
	};
} // namespace render