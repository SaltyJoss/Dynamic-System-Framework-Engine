// DSFE_GUI ViewportWidget.cpp
#include "Widgets/ViewportWidget.h"
#include "Simulation/SimulationManager.h"

#include <QVBoxLayout>
#include <QLabel>
#include <QPointer>

#include <QThread>
#include "Platform/KeyCode.h"

#ifdef _WIN32
    #include <windows.h>
#elif defined(__linux__)
    #include <QGuiApplication>
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
		renderer::NativeWindow n_win;
	#ifdef _WIN32
		n_win.handle = reinterpret_cast<void*>(winId());
	#elif defined(__linux__)
		// winId() is xcb_window_t; connection comes from Qt's X11 native interface
		n_win.handle = reinterpret_cast<void*>(static_cast<uintptr_t>(winId()));
		if (auto* x11 = qApp->nativeInterface<QNativeInterface::QX11Application>()) {
			n_win.connection = x11->connection();
		}
		else {
			LOG_ERROR("Failed to get X11 connection from Qt native interface; run with QT_QPA_PLATFORM=xcb");
			return;
		}
	#endif
		_sim->initialiseRenderer(n_win);
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
		if (!_sim) { return; }

        const qint64 now = _frameTimer.nsecsElapsed();
        const float dt = (_lastNs == 0) ? (1.0f / 144.0f) : static_cast<float>(now - _lastNs) * 1e-9f;
        _lastNs = now;

        _sim->tick(dt);
		_sim->renderViewport(width(), height());

        if (_mouse_captured) {
            static float smoothedDt = (1.0f / 144.0f);
            smoothedDt = glm::mix(smoothedDt, dt, 0.5f);
            _sim->handleContinuousMovement(_pressedKeys, smoothedDt);
        }
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