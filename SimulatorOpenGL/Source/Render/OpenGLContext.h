#pragma once

#include "RenderBase.h"
#include "UI/Styles.h"

namespace render {
	class OpenGLContext : public RenderContext {
	public:
		bool init(window::IWindow* window) override;
		void preRender() override;
		void postRender() override;
		void end() override;
	};
}