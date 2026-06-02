// DSFE_GUI ViewportWidget.cpp
#include "Widgets/ViewportWidget.h"

#include <QVBoxLayout>
#include <QLabel>

namespace widgets {
	ViewportWidget::ViewportWidget(QWidget* parent) : QOpenGLWidget(parent) {
	}
} // namespace widgets