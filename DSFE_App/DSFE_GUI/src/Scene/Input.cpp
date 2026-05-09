#include "pch.h"
// File:   Input.cpp
// GitHub: SaltyJoss
#include "Scene/Input.h"
#include <GLFW/glfw3.h>

#include "EngineLib/LogMacros.h"

using namespace scene;

// Get the currently pressed mouse button, if any
eInputButton Input::GetPressedButton(GLFWwindow* window) {
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)   { return eInputButton::Left; }
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)  { return eInputButton::Right; }
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS) { return eInputButton::Middle; }
    return eInputButton::None;
}

// Check if a specific key is currently pressed
bool Input::IsKeyPressed(GLFWwindow* window, int key) {
    return glfwGetKey(window, key) == GLFW_PRESS;
}

// Check if a specific mouse button is currently pressed
bool Input::IsMouseButtonPressed(GLFWwindow* window, eInputButton button) {
    int glfwButton;
    switch (button) {
    case eInputButton::Left:
        glfwButton = GLFW_MOUSE_BUTTON_LEFT;
        break;
    case eInputButton::Right:
        glfwButton = GLFW_MOUSE_BUTTON_RIGHT;
        break;
    case eInputButton::Middle:
        glfwButton = GLFW_MOUSE_BUTTON_MIDDLE;
        break;
    default:
        return false;
    }
    return glfwGetMouseButton(window, glfwButton) == GLFW_PRESS;
}