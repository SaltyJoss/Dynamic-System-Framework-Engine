// DSFE_GUI ViewportDock.cpp
#include "DockWidgets/ViewportDock.h"
#include "Widgets/ViewportWidget.h"

namespace dockwidgets {
	ViewportDock::ViewportDock(QWidget* parent) : QDockWidget("Viewport", parent) {
		setAllowedAreas(Qt::AllDockWidgetAreas);
		setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable);
		setWidget(new widgets::ViewportWidget(this));
	}
} // namespace dockwidgets