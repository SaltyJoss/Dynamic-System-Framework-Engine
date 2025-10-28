#include "DebugPanel.h"
#include <iostream>
#include <string>

void DebugPanel::Render(ImVec2 winSize, ImVec2 padding, float debugHeight, float ctrlPanelWidth) {
    ImGuiWindowFlags debugPanelFlags = 
          ImGuiWindowFlags_NoTitleBar
        | ImGuiWindowFlags_NoResize
        | ImGuiWindowFlags_NoCollapse
        | ImGuiWindowFlags_NoScrollbar
        | ImGuiWindowFlags_NoScrollWithMouse;

    ImGui::SetCursorPos(ImVec2(padding.x, winSize.y - debugHeight - padding.y));
    ImGui::BeginChild("Output", ImVec2(winSize.x - padding.x * 2, debugHeight), false, debugPanelFlags);
    if (ImGui::BeginTabBar("DEBUGGING")) {
        if (ImGui::BeginTabItem("Log")) {
            DebugLog();
            ImGui::EndTabItem();
        }
        
        if (ImGui::BeginTabItem("Error List")) {
            ErrorList();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::EndChild();
}

void DebugPanel::DebugLog() {
    ImGui::BeginChild("Debug Region", ImVec2(0, 0), true, ImGuiWindowFlags_AlwaysVerticalScrollbar);
        for (const auto& entry : entries)
        {
            ImVec4 col = entry.isError ? ImVec4(1, 0.2f, 0.2f, 1) : ImVec4(1, 1, 1, 1);
            ImGui::PushStyleColor(ImGuiCol_Text, col);
            ImGui::TextUnformatted(entry.text.c_str());
            ImGui::PopStyleColor();
        }
    ImGui::EndChild();

    if (autoScroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY())ImGui::SetScrollHereY(1.0f);
    
    if (ImGui::Button("Clear All")) entries.clear();
    ImGui::SameLine();
    if (ImGui::Button("Copy Log")) ImGui::LogToClipboard();
}

void DebugPanel::ErrorList() {
    ImGui::BeginChild("Error Region", ImVec2(0, 0), true, ImGuiWindowFlags_AlwaysVerticalScrollbar);
        for (const auto& entry : entries)
        {
            if (!entry.isError) continue;
            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, 0.2f, 0.2f, 1));
            ImGui::TextUnformatted(entry.text.c_str());
            ImGui::PopStyleColor();
        }
    ImGui::EndChild();
}

void DebugPanel::AddLog(const std::string& msg, bool error) {
	entries.push_back({msg, error});
}

ImVec4 DebugPanel::GetColour(bool error) const {
	return error ? ImVec4(1, 0, 0, 1) : ImVec4(1, 1, 1, 1);
}