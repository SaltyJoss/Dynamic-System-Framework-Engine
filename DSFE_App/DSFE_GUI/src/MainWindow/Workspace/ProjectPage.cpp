// DSFE_GUI ProjectPage.cpp
#include "Workspace/ProjectPage.h"
#include "Scene/SimulationManager.h"
#include "Widgets/ViewportWidget.h"
#include "Widgets/ControlPanelWidget.h"

#include <QLabel>
#include <QVBoxLayout>
#include <QSplitter>

namespace Workspace {
	ProjectPage::ProjectPage(gui::SimManager* sim, QWidget* parent) : QWidget(parent) {
		auto* layout = new QVBoxLayout(this);
		layout->setContentsMargins(0, 0, 0, 0);
		auto* splitter = new QSplitter(Qt::Horizontal, this);
		splitter->addWidget(new widgets::ViewportWidget(sim, splitter));
		splitter->addWidget(new widgets::ControlPanelWidget(sim, splitter));
		layout->addWidget(splitter);
		splitter->setStretchFactor(0, 4); // Viewport takes 4/5 of space
		splitter->setStretchFactor(1, 1); // Properties takes 1/5 of space
	}
} // namespace Workspace