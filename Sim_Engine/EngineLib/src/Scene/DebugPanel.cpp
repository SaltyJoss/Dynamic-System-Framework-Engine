
#include "pch.h"

#include <imgui.h>
#include "Scene/DebugPanel.h"
#include <io.h>

#include "EngineLib/LogMacros.h"

namespace gui {
    void gui::DebugPanel::render() {
        ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x - 310, ImGui::GetIO().DisplaySize.y - 200), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(300, 200), ImGuiCond_FirstUseEver);

        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.129f, 0.129f, 0.129f, 0.8f));
        ImGui::Begin("Output", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoTitleBar);

        beginDebugPanel("Output Panel");

        if (ImGui::BeginTabBar("Debug Tabs")) {
            if (ImGui::BeginTabItem("Log")) {
                renderLog();
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Error List")) {
                renderErrorTable();
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }

        endDebugPanel();

        ImGui::End();
        ImGui::PopStyleColor();
    }

    void DebugPanel::renderErrorTable() {
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 0.925f));
        ImGui::BeginChild("ErrorTableChild", ImVec2(0, 0), true, ImGuiWindowFlags_AlwaysVerticalScrollbar);
        
        const auto& entries = gLog.Instance().Entries();

        ImGuiTableFlags tableFlags =
            ImGuiTableFlags_BordersV |
            ImGuiTableFlags_BordersOuterH |
            ImGuiTableFlags_Resizable |
            ImGuiTableFlags_RowBg |
            ImGuiTableFlags_NoBordersInBody;

        if (ImGui::BeginTable("ErrorTable", 3, tableFlags))
        {
            ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn(" ", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("Description");
            ImGui::TableHeadersRow();

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);

            ImGuiTreeNodeFlags rootFlags =
                ImGuiTreeNodeFlags_SpanAllColumns |
                ImGuiTreeNodeFlags_DefaultOpen;

            bool openError = ImGui::TreeNodeEx("Errors:", rootFlags);
            bool openWarn = ImGui::TreeNodeEx("Warnings:", rootFlags);

            ImGui::TableSetColumnIndex(1);
            ImGui::TextUnformatted("");

            if (openError) {
                for (const auto& e : _entries) {
                    ImGui::TableNextRow();

                    // Column 0 -> level type
                    ImGui::TableSetColumnIndex(0);
                    ImGuiTreeNodeFlags leafFlags =
                        ImGuiTreeNodeFlags_Leaf |
                        ImGuiTreeNodeFlags_NoTreePushOnOpen |
                        ImGuiTreeNodeFlags_OpenOnArrow;

                    ImGui::Text(" ", leafFlags);

                    // Column 1 -> Description
                    ImGui::TableSetColumnIndex(1);
                    if (_entries.back().isError)
                        ImGui::TextColored({ 1,0,0,1 }, e.level.c_str());

                    ImGui::TableSetColumnIndex(2);
                    if (_entries.back().isError)
                        ImGui::TextUnformatted(e.desc.c_str());

                    ImGui::TableNextRow();
                }
                ImGui::TreePop();
            }

			ImGui::TableNextRow();

            if (openWarn) {
                for (const auto& e : _entries) {
                    ImGui::TableNextRow();

                    // Column 0 -> level type
                    ImGui::TableSetColumnIndex(0);
                    ImGuiTreeNodeFlags leafFlags =
                        ImGuiTreeNodeFlags_Leaf |
                        ImGuiTreeNodeFlags_NoTreePushOnOpen | 
                        ImGuiTreeNodeFlags_OpenOnArrow;

                    ImGui::TreeNodeEx(" ", leafFlags);

                    // Column 1 -> Description
                    ImGui::TableSetColumnIndex(1);
                    if (!_entries.back().isError)
                        ImGui::TextColored({ 1,1,0,1 }, e.level.c_str());

                    ImGui::TableSetColumnIndex(2);
                    if (!_entries.back().isError)
                        ImGui::TextUnformatted(e.desc.c_str());
                }
                ImGui::TreePop();
            }
            
            ImGui::EndTable();
        }

        ImGui::EndChild();
        ImGui::PopStyleColor();
    }

    void DebugPanel::renderLog() {
        ImGuiWindowFlags window_flags = ImGuiWindowFlags_HorizontalScrollbar 
                                      | ImGuiWindowFlags_AlwaysVerticalScrollbar;

        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 0.925f));
        ImGui::BeginChild("LogChild", ImVec2(0, -30), true, window_flags);

        const auto& entries = gLog.Instance().Entries();

        for (int i = 0; i < (int)entries.size(); ++i) {
			const auto& e = entries[i];

            const char* levelStr = "";
            ImVec4 levelColour{ 1, 1, 1, 1 }; // Colour associated log levelling! (I think its useful)

			// Determine log level string and colour
            switch (e.level) {
				// trace level for detailed debugging information
                case LogLevel::Trace:   levelStr = "TRACE";   levelColour = traceCol;   break;
                // debug level for general debugging information
                case LogLevel::Debug:   levelStr = "DEBUG";   levelColour = debugCol;   break;
                // info level for informational messages
                case LogLevel::Info:    levelStr = "INFO";    levelColour = infoCol;    break;
                // warning level for potential issues
                case LogLevel::Warning: levelStr = "WARN";    levelColour = warnCol;    break;
                // error level for error messages
                case LogLevel::Error:   levelStr = "ERROR";   levelColour = errorCol;   break;
                // success level for successful operations
                case LogLevel::Success: levelStr = "SUCCESS"; levelColour = okCol;      break;
                // fail level for failed operations
                case LogLevel::Fail:    levelStr = "FAIL";    levelColour = failCol;    break;
				// Runtime level for runtime specific messages
                case LogLevel::Runtime: levelStr = "RUNTIME"; levelColour = runtimeCol; break;
                // Output level for general output messages
                case LogLevel::Output:  levelStr = "OUTPUT";  levelColour = outputCol;  break;
                // Default level for general output messages
                default:                levelStr = "OUTPUT";  levelColour = outputCol;  break;
            }

            bool selected = selectedLines.count(i) > 0;

            ImGui::PushID(i);
            if (ImGui::Selectable("##logline", selected, ImGuiSelectableFlags_AllowDoubleClick | ImGuiSelectableFlags_SpanAllColumns)) {
                if (ImGui::GetIO().KeyShift && lastClickedLine != -1) {
                    int a = std::min(lastClickedLine, i);
                    int b = std::max(lastClickedLine, i);
                    selectedLines.clear();
                    for (int j = a; j <= b; ++j)
                        selectedLines.insert(j);
                }
                else if (ImGui::GetIO().KeyCtrl) {
                    if (selected) selectedLines.erase(i);
                    else selectedLines.insert(i);
                    lastClickedLine = i;
                }
                else {
                    selectedLines.clear();
                    selectedLines.insert(i);
                    lastClickedLine = i;
                }
            }

            // Render coloured text on top of selectable
            ImGui::SameLine();
            ImGui::TextUnformatted("[");
            ImGui::SameLine(0, 0);
            ImGui::TextColored(levelColour, "%s", levelStr);
            ImGui::SameLine(0, 0);
            ImGui::TextUnformatted("]: ");
            ImGui::SameLine(0, 0);
            ImGui::TextWrapped("%s", e.message.c_str());

            ImGui::PopID();
        }


        ImGui::EndChild();
        ImGui::PopStyleColor();

        if (ImGui::IsWindowFocused() && ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_C)) {
            std::string clip;
            for (int idx : selectedLines) {
                const auto& e = entries[idx];
                clip += e.message;
                clip += "\n";
            }
            ImGui::SetClipboardText(clip.c_str());
        }
    }

    void DebugPanel::beginDebugPanel(const char* id, ImVec2 size) {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 10.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8.0f, 5.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10.0f, 6.0f));
        ImGui::BeginChild(id, size, true, ImGuiWindowFlags_AlwaysUseWindowPadding);
    }

    void DebugPanel::endDebugPanel() {
        ImGui::EndChild();
        ImGui::PopStyleVar(4);
    }
}
