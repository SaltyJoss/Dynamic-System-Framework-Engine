// DSFE_GUI ProjectPage.h
#pragma once

#include <QWidget>

namespace gui { class SimulationManager; }
namespace widgets {
	class ConsoleOutputWidget;
	class DSLEditorWidget;
	class ControlPanelWidget;
}

namespace Workspace {
	class ProjectPage : public QWidget {
	public:
		explicit ProjectPage(gui::SimulationManager* sim, QWidget* parent = nullptr);
		widgets::DSLEditorWidget* editor() const;
		widgets::ConsoleOutputWidget* log() const;
		widgets::ControlPanelWidget* controlPanel() const;

	private:
		widgets::ConsoleOutputWidget* _log = nullptr;
		widgets::DSLEditorWidget* _editor = nullptr;
		widgets::ControlPanelWidget* _controlPanel = nullptr;
	};
} // namespace Workspace