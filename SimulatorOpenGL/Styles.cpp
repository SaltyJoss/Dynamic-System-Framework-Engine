#include "Styles.h"

void StyleModes::DarkMode() {
	// --- Rounding -- 
	style.WindowRounding = 0.0f;
	style.FrameRounding = 0.0f;
	style.ScrollbarRounding = 0.0f;
	style.GrabRounding = 0.0f;
	style.TabRounding = 0.0f;

	// --- Padding ---
	style.FramePadding = ImVec2(6, 4);
	style.ItemSpacing = ImVec2(6, 4);

	// --- Colours --- 
	colors[ImGuiCol_WindowBg] = ImVec4(0.12f, 0.12f, 0.12f, 1.0f);
	colors[ImGuiCol_ChildBg] = ImVec4(0.16f, 0.16f, 0.16f, 1.0f);
	colors[ImGuiCol_FrameBg] = ImVec4(0.18f, 0.18f, 0.18f, 1.0f);
	colors[ImGuiCol_FrameBgHovered] = ImVec4(0.28f, 0.28f, 0.28f, 1.0f);
	colors[ImGuiCol_FrameBgActive] = ImVec4(0.22f, 0.22f, 0.22f, 1.0f);
	colors[ImGuiCol_Button] = ImVec4(0.18f, 0.18f, 0.18f, 1.0f);
	colors[ImGuiCol_ButtonHovered] = ImVec4(0.28f, 0.28f, 0.28f, 1.0f);
	colors[ImGuiCol_ButtonActive] = ImVec4(0.22f, 0.22f, 0.22f, 1.0f);
	colors[ImGuiCol_Tab] = ImVec4(0.15f, 0.15f, 0.15f, 1.0f);
	colors[ImGuiCol_TabHovered] = ImVec4(0.28f, 0.28f, 0.28f, 1.0f);
	colors[ImGuiCol_TabActive] = ImVec4(0.22f, 0.22f, 0.22f, 1.0f);
	colors[ImGuiCol_TabUnfocused] = ImVec4(0.14f, 0.14f, 0.14f, 1.0f);
	colors[ImGuiCol_TabUnfocusedActive] = ImVec4(0.18f, 0.18f, 0.18f, 1.0f);
	colors[ImGuiCol_Border] = ImVec4(0.29f, 0.29f, 0.29f, 1.0f);

	// --- Headers ---
	colors[ImGuiCol_Header] = ImVec4(0.18f, 0.18f, 0.18f, 1.0f);
	colors[ImGuiCol_HeaderHovered] = ImVec4(0.28f, 0.28f, 0.28f, 1.0f);
	colors[ImGuiCol_HeaderActive] = ImVec4(0.22f, 0.22f, 0.22f, 1.0f);
}

void StyleModes::LightMode() {
	// --- Rounding --- 
	style.WindowRounding = 0.0f;
	style.FrameRounding = 0.0f;
	style.ScrollbarRounding = 0.0f;
	style.GrabRounding = 0.0f;
	style.TabRounding = 0.0f;

	// --- Padding ---
	style.FramePadding = ImVec2(6, 4);
	style.ItemSpacing = ImVec2(6, 4);

	// --- Colours ---
	colors[ImGuiCol_WindowBg] = ImVec4(0.94f, 0.94f, 0.94f, 1.0f);
	colors[ImGuiCol_ChildBg] = ImVec4(0.96f, 0.96f, 0.96f, 1.0f);
	colors[ImGuiCol_FrameBg] = ImVec4(0.90f, 0.90f, 0.90f, 1.0f);
	colors[ImGuiCol_FrameBgHovered] = ImVec4(0.82f, 0.82f, 0.82f, 1.0f);
	colors[ImGuiCol_FrameBgActive] = ImVec4(0.78f, 0.78f, 0.78f, 1.0f);
	colors[ImGuiCol_Button] = ImVec4(0.90f, 0.90f, 0.90f, 1.0f);
	colors[ImGuiCol_ButtonHovered] = ImVec4(0.82f, 0.82f, 0.82f, 1.0f);
	colors[ImGuiCol_ButtonActive] = ImVec4(0.78f, 0.78f, 0.78f, 1.0f);
	colors[ImGuiCol_Tab] = ImVec4(0.92f, 0.92f, 0.92f, 1.0f);
	colors[ImGuiCol_TabHovered] = ImVec4(0.82f, 0.82f, 0.82f, 1.0f);
	colors[ImGuiCol_TabActive] = ImVec4(0.78f, 0.78f, 0.78f, 1.0f);
	colors[ImGuiCol_TabUnfocused] = ImVec4(0.94f, 0.94f, 0.94f, 1.0f);
	colors[ImGuiCol_TabUnfocusedActive] = ImVec4(0.90f, 0.90f, 0.90f, 1.0f);
	colors[ImGuiCol_Border] = ImVec4(0.65f, 0.65f, 0.65f, 1.0f);

	// --- Headers ---
	colors[ImGuiCol_Header] = ImVec4(0.90f, 0.90f, 0.90f, 1.0f);
	colors[ImGuiCol_HeaderHovered] = ImVec4(0.82f, 0.82f, 0.82f, 1.0f);
	colors[ImGuiCol_HeaderActive] = ImVec4(0.78f, 0.78f, 0.78f, 1.0f);
}
