#ifndef CONTROL_PANEL_H
#define CONTROL_PANEL_H

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

class ControlPanel {
public:
    void Render(ImVec2 winSize, ImVec2 padding, float debugHeight, float ctrlPanelWidth);      // Draw the control panel UI using ImGui

private:
    void HandleInput(); // Process user input

    // Simulation controls
    void RenderSimulationControls();    // Start/Stop simulation, reset, speed control
    void RenderCameraControls();        // Switch POV modes
    void RenderObjectControls();        // Adjust object properties
    void RenderLinkControls();          // Adjust Link properties

    // Display settings
    void RenderDisplaySettings();       // Resolution, fullscreen logic

    // Debug/Info
    void RenderStats();     // FPS, errors, object count...
    void SimulationStats(const char* label, int* idx, float* velocity, float* torque, float* damping, float* position); // Simulation Time, Variable Change (Derivatives, Differentces), Link Count...

    // Internal state variables
    bool simulationRunning;     // Boolean of Simulation Runnning
    float simulationSpeed = 1;  // DEFAULT: 1x
    int povMode;    // 1 -> 4 {total 5 options}
    int idx;

    // Physics parameters
    float velocity = 0.0f;
    float torque = 0.0f;
    float linkLength = 1.0f;
    float damping = 0.1f;
    float position = 0.0f;

};

#endif