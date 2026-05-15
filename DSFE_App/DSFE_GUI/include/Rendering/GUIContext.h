// DSFE_Core GUIContext.h
#pragma once

#include "RenderBase.h"
#include "ui/Styles.h"
#include "Platform/Logger.h"

#include <functional>

namespace render {
	class GUIContext : public RenderContext {
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