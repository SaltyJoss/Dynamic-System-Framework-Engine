// DSFE_GUI ProjectPage.h
#pragma once

#include <QWidget>

namespace gui { class SimManager; }

namespace Workspace {
	class ProjectPage : public QWidget {
	public:
		explicit ProjectPage(gui::SimManager* sim, QWidget* parent = nullptr);
	};
} // namespace Workspace