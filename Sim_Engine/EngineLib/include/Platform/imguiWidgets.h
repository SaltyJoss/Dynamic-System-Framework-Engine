#pragma once
// File:   imguiWidgets.h
// GitHub: SaltyJoss

#include "EngineCore.h"

#include <imgui.h>
#include <imgui_internal.h>
#include "Platform/Logger.h"

namespace ImGui {
	// Draggable horizontal splitter for resizable panels
	static float hSplitter(const char* id, float* h, float minH = 200.0f, float maxH = 0.0f, float thickness = 2.0f) {
		if (maxH <= 0.0f) { maxH = GetContentRegionAvail().y; }

		// Get the current cursor position in screen coordinates
		ImVec2 cursor = GetCursorScreenPos();
		float w = GetContentRegionAvail().x;

		// Push a unique ID for this splitter to avoid conflicts with other widgets
		PushID(id);
		InvisibleButton("##hsplit", ImVec2(w, thickness));

		// Handle dragging
		bool dragging = IsItemActive();
		if (dragging) {
			*h += GetIO().MouseDelta.y;
		}

		*h = ImClamp(*h, minH, maxH);

		// Draw the draggable splitter
		if (IsItemHovered() || dragging) {
			SetMouseCursor(ImGuiMouseCursor_ResizeNS);
			ImVec2 p = GetItemRectMin();
			ImVec2 q = GetItemRectMax();
			GetWindowDrawList()->AddRectFilled(
				ImVec2(p.x, q.y), ImVec2(p.x, p.y + thickness),
				dragging ? IM_COL32(100, 150, 255, 200) // brighter color when dragging
						 : IM_COL32(150, 150, 150, 120) // default color
			);
		}

		// Restore cursor position and ID stack
		ImGui::Dummy(ImVec2(0, thickness));
		PopID();
		return *h;
	}

	// Draggable vertical splitter for resizable panels
	static float vSplitter(const char* id, float* w, float minW = 200.0f, float maxW = 0.0f, float thickness = 2.0f) {
		if (maxW <= 0.0f) { maxW = GetContentRegionAvail().x; }

		// Get the current cursor position in screen coordinates
		ImVec2 cursor = GetCursorScreenPos();
		float h = GetContentRegionAvail().y;

		// Push a unique ID for this splitter to avoid conflicts with other widgets
		PushID(id);
		SetCursorScreenPos(ImVec2(cursor.x + *w, cursor.y));
		InvisibleButton("##vsplit", ImVec2(thickness, h));

		// Handle dragging
		bool dragging = IsItemActive();
		if (dragging) {
			*w += GetIO().MouseDelta.x;
			*w = ImClamp(*w, minW, maxW);
		}

		// Draw the draggable splitter
		if (IsItemHovered() || dragging) {
			SetMouseCursor(ImGuiMouseCursor_ResizeEW);
			ImVec2 p = GetItemRectMin();
			ImVec2 q = GetItemRectMax();
			GetWindowDrawList()->AddRectFilled(
				ImVec2(p.x, p.y), ImVec2(p.x + thickness, q.y),
				dragging ? IM_COL32(100, 150, 255, 200) // brighter color when dragging
				: IM_COL32(150, 150, 150, 120) // default color
			);
		}

		// Restore cursor position and ID stack
		SetCursorScreenPos(cursor);
		PopID();
		return *w;
	}

	// Draw a section divider with spacing
	static void SectionDivider() {
		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
	}

	// Segmented Button Row Helper
	static bool SegmentedButtonRow(const char* label, const char* const* items, int itemCount, int& current, float buttonWidth) {
		ImGui::TextUnformatted(label);

		// Track if selection changed
		bool changed = false;
		ImGui::PushID(label);

		// Draw buttons in a row
		for (int i = 0; i < itemCount; ++i) {
			if (i > 0) ImGui::SameLine();

			// Highlight the currently selected button
			const bool selected = (current == i);
			if (selected) {
				ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.6f, 0.2f, 1.0f));
				ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.7f, 0.3f, 1.0f));
				ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.1f, 0.5f, 0.1f, 1.0f));
			}

			// Draw the button and check for clicks
			if (ImGui::Button(items[i], ImVec2(buttonWidth, 0))) {
				if (current != i) {
					current = i;
					changed = true;
				}
			}

			if (selected) { ImGui::PopStyleColor(3); }
		}

		ImGui::PopID();
		return changed;
	}

	// Reusable section header
	static void SectionHeader(const char* text, const ImVec4& color = ImVec4(1.0f, 1.0f, 1.0f, 1.0f)) {
		ImGui::Spacing();

		// Get draw list and position for custom drawing
		ImDrawList* dl = ImGui::GetWindowDrawList();
		ImVec2 p = ImGui::GetCursorScreenPos();

		// Subtle left accent bar
		dl->AddRectFilled(
			ImVec2(p.x, p.y),
			ImVec2(p.x + 3.0f, p.y + ImGui::GetTextLineHeightWithSpacing()),
			ImGui::ColorConvertFloat4ToU32(color)
		);

		// Header text with indentation
		ImGui::Indent(12.0f);
		ImGui::TextColored(color, "%s", text);
		ImGui::Unindent(12.0f);
		ImGui::Spacing();
	}
}

