// DSFE_GUI ViewportWidget.h
#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_5_Core>
#include <QTimer>
#include <QElapsedTimer>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QWheelEvent>

#include <unordered_set>

namespace gui { class SimManager; enum class eKeyCode; }

namespace widgets {
	class ViewportWidget : public QOpenGLWidget, protected QOpenGLFunctions_4_5_Core {
	public:
		explicit ViewportWidget(gui::SimManager* sim, QWidget* parent = nullptr);

	protected:
		void initializeGL() override;
		void resizeGL(int w, int h) override;
		void paintGL() override;

		void keyPressEvent(QKeyEvent* event) override;
		void keyReleaseEvent(QKeyEvent* event) override;

		void mousePressEvent(QMouseEvent* event) override;
		void mouseReleaseEvent(QMouseEvent* event) override;
		void mouseMoveEvent(QMouseEvent* event) override;

		void wheelEvent(QWheelEvent* event) override;

	private:
		gui::SimManager* _sim = nullptr;
		QElapsedTimer _frameTimer;
		qint64 _lastNs = 0;
		QTimer _updateTimer;
		QPoint _screenCenter;

		bool _mouseCaptured = false;
		std::unordered_set<gui::eKeyCode> _pressedKeys;
	};
}