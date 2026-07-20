// DSFE_GUI ViewportWidget.h
#pragma once

#include <QWidget>
#include <QTimer>
#include <QElapsedTimer>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QResizeEvent>
#include <QShowEvent>
#include <QPaintEvent>

#include <unordered_set>

namespace gui { class SimulationManager; enum class eKeyCode; }

namespace widgets {
	class ViewportWidget : public QWidget {
	public:
		explicit ViewportWidget(gui::SimulationManager* sim, QWidget* parent = nullptr);
		~ViewportWidget();

	protected:
		void showEvent(QShowEvent* event) override;
		void resizeEvent(QResizeEvent* event) override;
		void paintEvent(QPaintEvent* event) override;

		void keyPressEvent(QKeyEvent* event) override;
		void keyReleaseEvent(QKeyEvent* event) override;

		void mousePressEvent(QMouseEvent* event) override;
		void mouseReleaseEvent(QMouseEvent* event) override;
		void mouseMoveEvent(QMouseEvent* event) override;

		void wheelEvent(QWheelEvent* event) override;

		void initialise_renderer();

	private:
		gui::SimulationManager* _sim = nullptr;
		QElapsedTimer _frameTimer;
		qint64 _lastNs = 0;
		QTimer _updateTimer;
		bool _renderer_initialised = false;
		bool _mouse_captured = false;
		QPoint _screenCenter;
		std::unordered_set<gui::eKeyCode> _pressedKeys;
	};
}