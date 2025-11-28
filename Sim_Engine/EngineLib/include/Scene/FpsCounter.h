#pragma once

// =============================================
//            File: FpsCounter.h
// =============================================
// Class for tracking and calculating frames per second (FPS).
//
// Summary:
// =============================================
//
// public:
// --------------------------------------------
// void update()
//      -> Updates the FPS counter, should be called once per frame.
// double getFPS() const
//      -> Returns the current FPS value.
// --------------------------------------------
//
// private:
// --------------------------------------------
// bool init
//      -> Indicates whether the FPS counter has been initialized.
// double last
//      -> Timestamp of the last update.
// double accum
//      -> Accumulated time since the last FPS calculation.
// int frames
//      -> Number of frames counted since the last FPS calculation.
// double fps
//      -> Current FPS value.
// --------------------------------------------
//
// ============================================
//              GitHub: SaltyJoss
// ============================================

#include "EngineCore.h"
#include "Platform/Logger.h"
#include <GLFW/glfw3.h>

extern ENGINE_API Debug gLog;

namespace gui {
	class FpsCounter {
	public:
		void update() {
			double now = glfwGetTime();
			double dt = now - last;

			last = now;
			frames++;
			accum += dt;
			if (accum >= 1.0) {
				fps = frames / accum;
				frames = 0;
				accum = 0.0;
			}
		}

		double getFPS() const { return fps; }

	private:
		bool init = false;
		double last = glfwGetTime();
		double accum = 0.0;
		int frames = 0;
		double fps = 0.0;
	};
}