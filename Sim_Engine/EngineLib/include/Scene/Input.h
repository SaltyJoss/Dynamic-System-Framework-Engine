#pragma once

// =============================================
//            File: Input.h
// =============================================
// Class for handling input from keyboard and mouse.
//
// Summary:
// =============================================
//
// structures & enumeratiors:
// --------------------------------------------
// enum class eInputButton
//      -> Enumeration for mouse buttons (Left, Right, Middle, None).
// --------------------------------------------
// 
// public:
// --------------------------------------------
// eInputButton GetPressedButton(GLFWwindow* window)
//      -> Returns the currently pressed mouse button.
// bool IsKeyPressed(GLFWwindow* window, int key)
//      -> Checks if a specific key is pressed.
// bool IsMouseButtonPressed(GLFWwindow* window, eInputButton button)
//      -> Checks if a specific mouse button is pressed.
// --------------------------------------------
//
// private:
// --------------------------------------------
// Input()
//      -> Private constructor for the Input class.
// --------------------------------------------
//
// ============================================
//              GitHub: SaltyJoss
// ============================================


#include "EngineCore.h"

#include <GLFW/glfw3.h>
#include <cstdint>
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

namespace scene {
	enum class eInputButton {
		Left = 0,
		Right = 1,
		Middle = 2,
		None = 9
	};

	class ENGINE_API Input {
	public:
		static eInputButton GetPressedButton(GLFWwindow* window);

		static bool IsKeyPressed(GLFWwindow* window, int key);
		static bool IsMouseButtonPressed(GLFWwindow* window, eInputButton button);

	private:
		Input() = default;
	};
	
}