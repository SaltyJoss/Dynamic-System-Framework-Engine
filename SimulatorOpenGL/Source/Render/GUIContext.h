#pragma once

#include "RenderBase.h"
#include "UI/Styles.h"

namespace render {
	class GUIContext : public RenderContext {
	public:
		bool init(window::IWindow* win) override;
		void preRender() override;
		void postRender() override;
		void end() override;

	private:
		std::unique_ptr<gui::Styles> _style;
	};
}