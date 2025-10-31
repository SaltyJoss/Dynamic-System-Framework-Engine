#include "ch.h"

#include <imgui.h>

#include "DebugPanel.h"
#include <io.h>

void gui::DebugPanel::render() {
    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x - 310, ImGui::GetIO().DisplaySize.y - 200), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300, 200), ImGuiCond_FirstUseEver);
    ImGui::Begin("Debug Panel", nullptr, ImGuiWindowFlags_NoCollapse);

    if (ImGui::BeginTabBar("Debug Tabs")) {
        if (ImGui::BeginTabItem("Log")) {
            renderLog();
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Error List")) {
            renderErrors();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::End();
}

void gui::DebugPanel::renderLog() {
    ImGui::BeginChild("LogChild", ImVec2(0, -30), true, ImGuiWindowFlags_AlwaysVerticalScrollbar);
    for (const auto& entry : entries) {
        ImVec4 col = entry.isError ? ImVec4(1, 0.3f, 0.3f, 1) : ImVec4(1, 1, 1, 1);
        ImGui::PushStyleColor(ImGuiCol_Text, col);
        ImGui::TextUnformatted(entry.text.c_str());
        ImGui::PopStyleColor();
    }
    ImGui::EndChild();

    if (ImGui::Button("Clear")) entries.clear();
    ImGui::SameLine();
    if (ImGui::Button("Copy")) ImGui::LogToClipboard();
}

void gui::DebugPanel::renderErrors() {
    ImGui::BeginChild("ErrorChild", ImVec2(0, 0), true, ImGuiWindowFlags_AlwaysVerticalScrollbar);
    for (const auto& entry : entries)
        if (entry.isError)
            ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "%s", entry.text.c_str());
    ImGui::EndChild();
}

void gui::DebugPanel::addLog(const std::string& msg, bool error) { entries.push_back({ msg, error }); }
ImVec4 gui::DebugPanel::getColour(bool error) const { return error ? ImVec4(1, 0, 0, 1) : ImVec4(1, 1, 1, 1); }

/*
OLD LOGIC -- DELETE SOON, KEEP WHILE STILL TUNING NEW LOGIC

void DebugPanel::Render(ImVec2 winSize, ImVec2 padding, float debugHeight, float ctrlPanelWidth) {
    ImGuiWindowFlags debugPanelFlags = 
          ImGuiWindowFlags_NoTitleBar
        | ImGuiWindowFlags_NoResize
        | ImGuiWindowFlags_NoCollapse
        | ImGuiWindowFlags_NoScrollbar
        | ImGuiWindowFlags_NoScrollWithMouse;

    ImGui::SetCursorPos(ImVec2(padding.x, winSize.y - debugHeight - padding.y));
    ImGui::BeginChild("Output", ImVec2(winSize.x - padding.x * 2, debugHeight), false, debugPanelFlags);*/

   /* float tabWidth = (ctrlPanelWidth / 2 - padding.x * 2) / 2.0f;
    int currentDebugTab=0;

    if (ImGui::Button("Output", ImVec2(tabWidth, 25))) currentDebugTab = 0;
    ImGui::SameLine();
    if (ImGui::Button("Error List", ImVec2(tabWidth, 25))) currentDebugTab = 1;

    ImGui::Separator(); // optional line under tabs

    // Draw selected content
    if (currentDebugTab == 0) DebugLog();
    else if (currentDebugTab == 1) ErrorList();*/

    /*if (ImGui::BeginTabBar("DEBUGGING")) {
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
}*/
