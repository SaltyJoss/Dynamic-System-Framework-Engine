// DSFE_GUI ViewportWidget.cpp
#include "Widgets/ViewportWidget.h"
#include "Simulation/SimulationManager.h"

#include <QVBoxLayout>
#include <QLabel>
#include <QPointer>
#include <QPaintEngine>

#include <QThread>
#include "Platform/KeyCode.h"

#ifdef _WIN32
    #include <windows.h>
#endif

namespace widgets {
	ViewportWidget::ViewportWidget(gui::SimulationManager* sim, QWidget* parent) : QWidget(parent), _sim(sim) {
		setAttribute(Qt::WA_NativeWindow);
		setAttribute(Qt::WA_PaintOnScreen);      // Qt won't touch the pixels
		setAttribute(Qt::WA_NoSystemBackground);
		setAttribute(Qt::WA_OpaquePaintEvent);

		setFocusPolicy(Qt::StrongFocus);
		setMouseTracking(true);

		connect(&_updateTimer, &QTimer::timeout, this, [this]() {update(); });
		_updateTimer.start(7); // ~144 FPS
	}

	ViewportWidget::~ViewportWidget() {}

	void ViewportWidget::initialise_renderer() {
	#ifdef _WIN32
		HWND hwnd = reinterpret_cast<HWND>(winId());
		_sim->initialiseRenderer(static_cast<void*>(hwnd));
	#elif defined(__linux__)
		auto handle = winId();
		_sim->initialiseRenderer(reinterpret_cast<void*>(handle));
	#endif
		_renderer_initialised = true;
	}

	void ViewportWidget::showEvent(QShowEvent* event) {
		QWidget::showEvent(event);
		if (!_renderer_initialised && width() > 1 && height() > 1) {
			initialise_renderer();
		}
	}

	void ViewportWidget::resizeEvent(QResizeEvent* event) {
		QWidget::resizeEvent(event);
		if (!_renderer_initialised && isVisible() && width() > 1 && height() > 1) {
			initialise_renderer();
			return;
		}
		if (_sim) { _sim->resizeRenderer(width(), height()); }
	}

	void ViewportWidget::paintEvent(QPaintEvent* event) {
		if (_sim) { _sim->renderViewport(width(), height()); } // Need to add to SimulationManager
	}

	void ViewportWidget::keyPressEvent(QKeyEvent* event) {
		switch (event->key()) {
			case Qt::Key_W: _pressedKeys.insert(gui::eKeyCode::W); break;
			case Qt::Key_A: _pressedKeys.insert(gui::eKeyCode::A); break;
			case Qt::Key_S: _pressedKeys.insert(gui::eKeyCode::S); break;
			case Qt::Key_D: _pressedKeys.insert(gui::eKeyCode::D); break;
			case Qt::Key_Space: _pressedKeys.insert(gui::eKeyCode::Space); break;
			case Qt::Key_Control: _pressedKeys.insert(gui::eKeyCode::Ctrl); break;
			case Qt::Key_Shift: _pressedKeys.insert(gui::eKeyCode::LShift); break;
			case Qt::Key_Escape:
				_mouse_captured = !_mouse_captured;
				if (_mouse_captured) {
					setFocus();
					setCursor(Qt::BlankCursor);
					_screenCenter = mapToGlobal(rect().center());
					QCursor::setPos(_screenCenter);
					grabMouse();
					if (_sim) { _sim->resetMouseDelta(); }
				}
				else {
					releaseMouse();
					unsetCursor();
					if (_sim) { _sim->resetMouseDelta(); }
				}
				break;
		}
		QWidget::keyPressEvent(event);
	}

	void ViewportWidget::keyReleaseEvent(QKeyEvent* event) {
		switch (event->key()) {
			case Qt::Key_W: _pressedKeys.erase(gui::eKeyCode::W); break;
			case Qt::Key_A: _pressedKeys.erase(gui::eKeyCode::A); break;
			case Qt::Key_S: _pressedKeys.erase(gui::eKeyCode::S); break;
			case Qt::Key_D: _pressedKeys.erase(gui::eKeyCode::D); break;
			case Qt::Key_Space: _pressedKeys.erase(gui::eKeyCode::Space); break;
			case Qt::Key_Control: _pressedKeys.erase(gui::eKeyCode::Ctrl); break;
			case Qt::Key_Shift: _pressedKeys.erase(gui::eKeyCode::LShift); break;
		}
		QWidget::keyReleaseEvent(event);
	}

	void ViewportWidget::mousePressEvent(QMouseEvent* event) {
		if (event->button() == Qt::RightButton) {
			_mouse_captured = true;
			setFocus();
			setCursor(Qt::BlankCursor);
			QCursor::setPos(mapToGlobal(rect().center()));
			grabMouse();
			if (_sim) { _sim->resetMouseDelta(); }
		}
	}

	void ViewportWidget::mouseReleaseEvent(QMouseEvent* event) {
		if (event->button() == Qt::RightButton) {
			_mouse_captured = false;
			releaseMouse();
			unsetCursor();
		}
	}

	void ViewportWidget::mouseMoveEvent(QMouseEvent* event) {
		if (!_sim || !_mouse_captured) { return; }
		QPoint current = QCursor::pos();
		QPoint delta = current - _screenCenter;
		_sim->handleMouseLook(delta.x(), -delta.y(), true);
		QCursor::setPos(_screenCenter);
	}

	void ViewportWidget::wheelEvent(QWheelEvent* event) {
		if (!_sim) { return; }
		_sim->onMouseWheel(event->angleDelta().y() / 120.0);
	}

} // namespace widgets