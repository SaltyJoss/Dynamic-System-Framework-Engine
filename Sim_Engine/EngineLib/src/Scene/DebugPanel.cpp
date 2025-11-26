
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
        ImGui::Begin("Output", nullptr, ImGuiWindowFlags_NoCollapse);

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
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 0.925f));
        ImGui::BeginChild("LogChild", ImVec2(0, -30), true, ImGuiWindowFlags_AlwaysVerticalScrollbar);

        const auto& entries = gLog.Instance().Entries();

        for (const auto& e : entries) {
            const char* levelStr = "";
            ImVec4 levelColour{ 1, 1, 1, 1 }; // Colour associated log levelling! (I think its useful)
            debugLogEntry _e;

            switch (e.level) {
            // Trace level for very detailed logging (e.g. function entries/exits, loop iterations)
            case LogLevel::Trace:
                levelStr = "TRACE";
				levelColour = traceCol;
                break;
            // Debug level for detailed debugging information (e.g. variable values, function calls)
            case LogLevel::Debug:
                levelStr = "DEBUG";
				levelColour = debugCol;
                break;
            // Info level for general information (e.g. startup messages, asset loaded)
            case LogLevel::Info:
                levelStr = "INFO";
				levelColour = infoCol;
                break;
            // Warning level for non-critical issues (e.g. deprecated function, bad input)
            case LogLevel::Warning:
                levelStr = "WARN";
				levelColour = warnCol;
                _e.level = "WARN";
                _e.desc = e.message;
				_e.isError = false;
                _entries.push_back(_e);
                break;
            // Error level for critical issues (e.g. failed to initialize, crashed)
            case LogLevel::Error:
                levelStr = "ERROR";
				levelColour = errorCol;
				_e.level = "ERROR";
                _e.desc = e.message;
				_e.isError = true;
				_entries.push_back(_e);
                break;
            // Ok level for successful operations (e.g. asset loaded, sim completed)
            case LogLevel::Ok:
                levelStr = "OK";
				levelColour = okCol;
                break;
            // Fail level for runtime failures (e.g. failed to load asset, sim failed to complete)
            case LogLevel::Fail:
                levelStr = "FAIL";
				levelColour = failCol;
                break;
			// Runtime level for runtime messages (e.g. performance, sim time)
			case LogLevel::Runtime:
				levelStr = "RUNTIME";
				levelColour = runtimeCol;
				break;
            // Output level for general output messages
            case LogLevel::Output:
                levelStr = "OUTPUT";
                levelColour = outputCol;
                break;
            // Default level for general output messages
            default:
                levelStr = "OUTPUT";
				levelColour = outputCol;
                break;
            }

            // Render log entry (level and message ~ formatted)
            ImGui::TextWrapped("[");
            ImGui::SameLine();
            ImGui::TextColored(levelColour, "%s", levelStr);
            ImGui::SameLine();
            ImGui::TextWrapped("]: %s", e.message.c_str());
        }

        ImGui::EndChild();
        ImGui::PopStyleColor();

        if (ImGui::Button("Clear"))
            ImGui::SameLine();
        if (ImGui::Button("Copy")) ImGui::LogToClipboard();
    }
}
