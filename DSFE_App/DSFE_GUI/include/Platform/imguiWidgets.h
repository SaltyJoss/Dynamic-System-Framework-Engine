// DSFE_GUI imguiWidgets.h
#pragma once

#include <imgui.h>
#include <imgui_internal.h>
#include "Platform/Logger.h"

namespace ImGui {
	// Draggable horizontal splitter for resizable panels
	inline float hSplitter(const char* id, float* h, float minH = 200.0f, float maxH = 0.0f, float thickness = 2.0f) {
		if (maxH <= 0.0f) { maxH = GetContentRegionAvail().y; }

		// Get the current cursor position in screen coordinates
		ImVec2 cursor = GetCursorScreenPos();
		float w = GetContentRegionAvail().x;

		// Push a unique ID for this splitter to avoid conflicts with other widgets
		PushID(id);

		// Position the invisible button for the splitter
		if (w <= 0.0f) {
			ImGui::Dummy(ImVec2(0.0f, thickness));
			PopID();
			return *h;
		}

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
		if (w > 0.0f) { ImGui::Dummy(ImVec2(0, thickness)); }
		PopID();
		return *h;
	}

	// Draggable vertical splitter for resizable panels
	inline float vSplitter(const char* id, float* w, float minW = 200.0f, float maxW = 0.0f, float thickness = 2.0f) {
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
	inline void SectionDivider() {
		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
	}

	// Segmented Button Row Helper
	inline bool SegmentedButtonRow(const char* label, const char* const* items, int itemCount, int& current, float buttonWidth) {
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
	inline void SectionHeader(const char* text, const ImVec4& color = ImVec4(1.0f, 1.0f, 1.0f, 1.0f)) {
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

	// Draw a simple fraction (numerator over denominator) with a horizontal line in between
	inline void DrawFraction(const char* numerator, const char* denom) {
		ImDrawList* draw = ImGui::GetWindowDrawList();
		ImVec2 pos = ImGui::GetCursorScreenPos();

		// Calculate text sizes
		ImVec2 numSize = ImGui::CalcTextSize(numerator);
		ImVec2 denSize = ImGui::CalcTextSize(denom);

		// Calculate positions
		float lineY = pos.y + numSize.y + 2.0f;
		float width = std::max(numSize.x, denSize.x) + 10.0f;

		// Center numerator
		draw->AddText(
			ImVec2(pos.x + (width - numSize.x) * 0.5f, pos.y),
			IM_COL32_WHITE, numerator
		);

		// Draw fraction bar
		draw->AddLine(
			ImVec2(pos.x, lineY), ImVec2(pos.x + width, lineY),
			IM_COL32_WHITE, 1.5f
		);

		// Center denominator
		draw->AddText(
			ImVec2(pos.x + (width - denSize.x) * 0.5f, lineY + 2.0f),
			IM_COL32_WHITE, denom
		);

		// Advance cursor so layout continues correctly
		ImGui::Dummy(ImVec2(width, numSize.y + denSize.y + 6.0f));
	}

	// Interactive fraction where the denominator is 60 * k, and k can be adjusted by dragging horizontally
	inline bool DragDtFraction(const char* id, int& k, bool isTelem = false) {
		ImDrawList* draw = ImGui::GetWindowDrawList();
		ImVec2 pos = ImGui::GetCursorScreenPos();

		// Ensure k is at least 1 to avoid zero or negative denominators
		k = std::max(k, 1);
		int denom = 30 * k;

		// Limits for k to prevent unreasonable values
		const int MIN_K = 1;
		const int MAX_K = isTelem ? 20 : 800;

		// Format denominator text
		char denomBuf[32];
		snprintf(denomBuf, sizeof(denomBuf), "%d", denom);

		// Calculate text sizes
		ImVec2 numSize = ImGui::CalcTextSize("1");
		ImVec2 denSize = ImGui::CalcTextSize(denomBuf);

		// Calculate width and positions
		float width = std::max(numSize.x, denSize.x) + 10.0f;
		float lineY = pos.y + numSize.y + 2.0f;
		float totalHeight = numSize.y + denSize.y + 6.0f;

		// Numerator
		draw->AddText(
			ImVec2(pos.x + (width - numSize.x) * 0.5f, pos.y),
			IM_COL32_WHITE, "1"
		);

		// Line
		draw->AddLine(
			ImVec2(pos.x, lineY), ImVec2(pos.x + width, lineY),
			IM_COL32_WHITE, 1.5f
		);

		// Denominator position
		ImVec2 denPos(pos.x + (width - denSize.x) * 0.5f, lineY + 2.0f);

		// Interactive region
		ImGui::SetCursorScreenPos(denPos);
		ImVec2 hitSize(denSize.x + 10.0f, denSize.y + 6.0f);
		ImGui::InvisibleButton(id, hitSize);

		// Determine color based on interaction state
		ImU32 colour = ImGui::IsItemActive()
			? IM_COL32(255, 220, 120, 255) // active color
			: ImGui::IsItemHovered()
			? IM_COL32(200, 200, 255, 255) // hover color
			: IM_COL32_WHITE;

		// Draw denominator with state-aware color
		draw->AddText(denPos, colour, denomBuf);

		bool changed = false;
		static float dragAccum = 0.0f;

		if (ImGui::IsItemActive()) {
			ImGuiIO& io = ImGui::GetIO();
			dragAccum += io.MouseDelta.x;
			int step = (int)dragAccum;

			// Update k if drag has accumulated enough to cross a step threshold
			if (step != 0) {
				dragAccum -= step * 1.0f;
				int newK = k + step;
				newK = std::clamp(newK, MIN_K, MAX_K);
				// Only update if k actually changes
				if (newK != k) {
					k = newK;
					changed = true;
				}
			}
		}
		else { dragAccum = 0.0f; }

		// Also allow adjusting k with the mouse wheel when hovering
		if (ImGui::IsItemHovered()) {
			ImGuiIO& io = ImGui::GetIO();

			// Mouse wheel typically scrolls vertically, but we can interpret it as horizontal adjustment for this widget
			if (io.MouseWheel != 0.0f) {
				int newK = k + (int)io.MouseWheel;
				newK = std::clamp(newK, MIN_K, MAX_K);
				// Only update if k actually changes
				if (newK != k) {
					k = newK;
					changed = true;
				}

				// Reset mouse wheel to prevent affecting other widgets
				io.MouseWheel = 0.0f;
			}
		}

		ImGui::NewLine();
		float dt = 1.0f / (float)denom;
		ImGui::Text("dt: %.6f s", dt);

		// Advance cursor to account for the space taken by the fraction
		ImGui::Dummy(ImVec2(width, totalHeight));
		return changed;
	}
}

