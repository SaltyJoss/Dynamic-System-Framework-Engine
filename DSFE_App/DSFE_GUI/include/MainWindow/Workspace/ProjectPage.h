// DSFE_GUI ProjectPage.h
#pragma once

#include <QWidget>

namespace gui { class SimManager; }
namespace widgets {
	class ConsoleOutputWidget;
	class DSLEditorWidget;
}

namespace Workspace {
	class ProjectPage : public QWidget {
	public:
		explicit ProjectPage(gui::SimManager* sim, QWidget* parent = nullptr);
		widgets::DSLEditorWidget* editor() const;

	private:
		widgets::ConsoleOutputWidget* _log = nullptr;
		widgets::DSLEditorWidget* _editor = nullptr;
	};
} // namespace Workspace