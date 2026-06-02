// DSFE_GUI ViewportDock.h
#pragma once

#include <QDockWidget>

class ViewportWidget;

namespace dockwidgets {
	class ViewportDock : public QDockWidget {
	public:
		explicit ViewportDock(QWidget* parent = nullptr);
	};
} // namespace dockwidgets