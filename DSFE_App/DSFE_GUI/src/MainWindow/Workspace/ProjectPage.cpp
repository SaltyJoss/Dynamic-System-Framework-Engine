// DSFE_GUI ProjectPage.cpp
#include "Workspace/ProjectPage.h"
#include "Scene/SimulationManager.h"
#include "Widgets/ViewportWidget.h"
#include "Widgets/ControlPanelWidget.h"
#include "Widgets/DSLEditorWidget.h"

#include <QLabel>
#include <QVBoxLayout>
#include <QSplitter>

namespace Workspace {
	ProjectPage::ProjectPage(gui::SimManager* sim, QWidget* parent) : QWidget(parent) {
		auto* layout = new QVBoxLayout(this);
		layout->setContentsMargins(0, 0, 0, 0);
		auto* rootSplitter = new QSplitter(Qt::Horizontal, this);
		auto* centreSplitter = new QSplitter(Qt::Vertical);
		auto* rightSplitter = new QSplitter(Qt::Vertical);
		_editor = new widgets::DSLEditorWidget(sim, this);
		rootSplitter->addWidget(_editor);
		centreSplitter->addWidget(new widgets::ViewportWidget(sim, this));
		centreSplitter->addWidget(new QLabel("Console Output (TODO)", this));
		rightSplitter->addWidget(new widgets::ControlPanelWidget(sim, this));
		rightSplitter->addWidget(new QLabel("Scene Object? (TODO)", this));
		rootSplitter->addWidget(centreSplitter);
		rootSplitter->addWidget(rightSplitter);
		layout->addWidget(rootSplitter);
		// Sizing reused from the old imgui layout
		rootSplitter->setSizes({ 635, 1016, 393 });
		centreSplitter->setSizes({ 733, 396 });
		rightSplitter->setSizes({ 733, 396 });
	}

	widgets::DSLEditorWidget* ProjectPage::editor() const { return _editor; }

} // namespace Workspace