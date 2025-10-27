#ifndef CONTROL_PANEL_H
#define CONTROL_PANEL_H

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

class ControlPanel {
public:
    void Render();                  // Draw the control panel UI using ImGui

private:
    void HandleInput();             // Process user input

    // Simulation controls
    void RenderSimulationControls();// Start/Stop simulation, reset, speed control
    void RenderCameraControls();    // Switch POV modes
    void RenderObjectControls();    // Adjust object properties
    void RenderLinkControls();       // Adjust Link properties

    // Display settings
    void RenderDisplaySettings();   // Resolution, fullscreen logic

    // Debug/Info
    void RenderStats();             // FPS, simulation time, object count

    // Internal state variables
    bool simulationRunning;
    int povMode;    // 1 -> 4
    float simulationSpeed;

    // Physics parameters
    float velocity = 0.0f;
    float torque = 0.0f;
    float linkLength = 1.0f;
    float damping = 0.1f;
};

#endif