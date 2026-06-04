// DSFE_GUI SimUI.cpp
#include "Scene/SimulationManager.h"
#ifdef __gl_h_
#undef __gl_h_
#endif
#include "Manager/SimImplementation.h"

#include <imgui.h>

namespace gui {
	// Main Dockspace with Menu Bar
	void SimManager::drawMainDockspace() {
		ImGuiWindowFlags flags =
			ImGuiWindowFlags_NoDocking |
			ImGuiWindowFlags_NoTitleBar |
			ImGuiWindowFlags_NoCollapse |
			ImGuiWindowFlags_NoResize |
			ImGuiWindowFlags_NoMove |
			ImGuiWindowFlags_NoBringToFrontOnFocus |
			ImGuiWindowFlags_NoNavFocus;

		const ImGuiViewport* vp = ImGui::GetMainViewport();
		ImGui::SetNextWindowPos(vp->Pos);
		ImGui::SetNextWindowSize(vp->Size);
		ImGui::SetNextWindowViewport(vp->ID);

		ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
		ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);

		// Must be a window so DockSpace has somewhere to live
		ImGui::Begin("##MainDockspace", nullptr, flags);

		ImGui::PopStyleVar(2);

		beginSimManager("##MainDockspaceChild");

		ImGuiID dock_id = ImGui::GetID("MainDockspaceID");
		ImGui::DockSpace(dock_id, ImVec2(0, 0), ImGuiDockNodeFlags_PassthruCentralNode);

		endSimManager();

		ImGui::End();
	}

	// Viewport Window
	void SimManager::drawViewportWindow() {
		ImGui::Begin("Viewport", nullptr,
			ImGuiWindowFlags_NoScrollbar |
			ImGuiWindowFlags_NoScrollWithMouse);

		beginSimManager("##ViewportBody");

		// --- Tabs: Single / Quad ---
		if (ImGui::BeginTabBar("ViewportTabs", ImGuiTabBarFlags_None)) {
			const bool singleSelected = ImGui::BeginTabItem("Single");
			if (singleSelected) {
				// Restore to Manual view when switching back from Quad
				if (_impl->viewMode == Impl::ViewMode::Quad) {
					_impl->activeView = gui::ViewID::Manual;
				}
				_impl->viewMode = Impl::ViewMode::Single;
				ImGui::EndTabItem();
			}

			const bool quadSelected = ImGui::BeginTabItem("Quad");
			if (quadSelected) {
				_impl->viewMode = Impl::ViewMode::Quad;
				ImGui::EndTabItem();
			}

			ImGui::EndTabBar();
		}

		// Everything below tabs is render output
		_isHovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows);

		ImVec2 panel = ImGui::GetContentRegionAvail();

		// Pixel size (framebuffer coords)
		int vpW = (int)(panel.x);
		int vpH = (int)(panel.y);
		vpW = std::max(1, vpW);
		vpH = std::max(1, vpH);

		// Update DISPLAY size only
		if ((int)_displaySize.x != vpW || (int)_displaySize.y != vpH) {
			_displaySize = { (float)vpW, (float)vpH };

			// invalidate only display cached sizes so post buffers resize
			for (auto& v : _impl->_views) {
				v.displayW = 0;
				v.displayH = 0;
			}
			//LOG_INFO("Viewport display size updated to %dx%d", vpW, vpH);
		}

		// --- Render + Present ---
		if (_impl->viewMode == Impl::ViewMode::Quad) {
			int halfW = std::max(1, vpW / 2);
			int halfH = std::max(1, vpH / 2);

			_impl->renderView(*this, _impl->_views[(size_t)gui::ViewID::Top], halfW, halfH);
			_impl->renderView(*this, _impl->_views[(size_t)gui::ViewID::Front], halfW, halfH);
			_impl->renderView(*this, _impl->_views[(size_t)gui::ViewID::Right], halfW, halfH);
			_impl->renderView(*this, _impl->_views[(size_t)gui::ViewID::Follow], halfW, halfH);

			// Stable 2x2 layout
			ImVec2 avail = ImGui::GetContentRegionAvail();
			ImVec2 cell = ImVec2(avail.x * 0.5f, avail.y * 0.5f);

			// Helper to draw each cell with the same pattern
			auto drawCell = [&](const char* childId, gui::ViewID id, bool sameLine) {
				if (sameLine) ImGui::SameLine();
				ImGui::BeginChild(childId, cell, false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

				auto& v = _impl->_views[(size_t)id];
				ImVec2 inner = ImGui::GetContentRegionAvail();

				ImGui::Image((ImTextureID)(intptr_t)v.post->getTexture(), inner, ImVec2(0, 1), ImVec2(1, 0));

				// Set active view when clicking inside the quad cell
				if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
					_impl->activeView = id;
				}

				// Handle scroll zoom on hovered quad cell
				if (ImGui::IsWindowHovered()) {
					float scrollY = ImGui::GetIO().MouseWheel;
					if (scrollY != 0.0f) {
						v.cam->onMouseWheel((double)scrollY);
					}
				}

				// Visual indication: draw a border around the active cell
				if (_impl->activeView == id) {
					ImDrawList* dl = ImGui::GetWindowDrawList();
					ImVec2 p0 = ImGui::GetWindowPos();
					ImVec2 p1 = ImVec2(p0.x + ImGui::GetWindowSize().x, p0.y + ImGui::GetWindowSize().y);
					dl->AddRect(p0, p1, IM_COL32(255, 200, 0, 180), 4.0f, 0, 2.0f);
				}

				ImGui::EndChild();
			};

			drawCell("##Top", gui::ViewID::Top, false);
			drawCell("##Front", gui::ViewID::Front, true);
			drawCell("##Right", gui::ViewID::Right, false);
			drawCell("##Follow", gui::ViewID::Follow, true);
		}
		else {
			auto& v = _impl->_views[static_cast<size_t>(_impl->activeView)];
			_impl->renderView(*this, v, vpW, vpH);

			ImGui::Image((ImTextureID)(intptr_t)v.post->getTexture(), panel, ImVec2(0, 1), ImVec2(1, 0));
		}

		endSimManager();

		ImGui::End();
	}

	// Begin Control Panel Helper
	void SimManager::beginSimManager(const char* id) {
		ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 10.0f));
		ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
		ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8.0f, 5.0f));
		ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10.0f, 8.0f));

		ImGui::BeginChild(id, ImVec2(0, 0), true,
			ImGuiChildFlags_AlwaysUseWindowPadding |
			ImGuiWindowFlags_NoScrollbar |
			ImGuiWindowFlags_NoScrollWithMouse);
	}

	// End Control Panel Helper
	void SimManager::endSimManager() {
		ImGui::EndChild();
		ImGui::PopStyleVar(4);
	}
}