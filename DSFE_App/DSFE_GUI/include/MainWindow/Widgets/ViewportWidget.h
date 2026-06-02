// DSFE_GUI ViewportWidget.h
#pragma once

#include <QOpenGLWidget>

namespace widgets {
	class ViewportWidget : public QOpenGLWidget {
	public:
		explicit ViewportWidget(QWidget* parent = nullptr);
	};
}