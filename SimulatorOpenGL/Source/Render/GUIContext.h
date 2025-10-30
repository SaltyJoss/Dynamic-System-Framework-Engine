#pragma once

#include "RenderBase.h"

namespace render {
	class GUIContext : public RenderContext {
	public:
		bool init(window::IWindow* win) override;
		void preRender() override;
		void postRender() override;
		void end() override;
	};
}