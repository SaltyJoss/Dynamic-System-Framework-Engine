#pragma once

#include "RenderBase.h"

namespace render {
	class OpenGLContext : public RenderContext {
	public:
		bool init(window::IWindow* win) override;
		void preRender() override;
		void postRender() override;
		void end() override;
	};
}