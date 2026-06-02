// DSFE_GUI ProjectPage.cpp
#include "Workspace/ProjectPage.h"
#include "Widgets/ViewportWidget.h"

#include <QLabel>
#include <QVBoxLayout>
#include <QSplitter>

namespace Workspace {
	ProjectPage::ProjectPage(QWidget* parent) : QWidget(parent) {
		auto* layout = new QVBoxLayout(this);
		auto* splitter = new QSplitter(Qt::Horizontal, this);
		splitter->addWidget(new widgets::ViewportWidget(splitter));
		splitter->addWidget(new QLabel("Properties(PlaceHolder)", splitter));
		splitter->setStretchFactor(0, 4); // Viewport takes 4/5 of space
		splitter->setStretchFactor(1, 1); // Properties takes 1/5 of space
		layout->addWidget(splitter);
	}
} // namespace Workspace