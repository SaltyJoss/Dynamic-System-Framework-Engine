// DSFE_GUI ViewportWidget.cpp
#include <glad/glad.h>

#include "Widgets/ViewportWidget.h"
#include "Scene/SimulationManager.h"

#include <QOpenGLContext>

#include <QVBoxLayout>
#include <QLabel>
#include <QPointer>

#include <QThread>
#include "Platform/KeyCode.h"

namespace widgets {
	ViewportWidget::ViewportWidget(gui::SimManager* sim, QWidget* parent) : QOpenGLWidget(parent), _sim(sim) {
		setFocusPolicy(Qt::StrongFocus);
		setMouseTracking(true);

		connect(&_updateTimer, &QTimer::timeout, this, [this]() {update(); });
		_updateTimer.start(7); // ~144 FPS
	}

	ViewportWidget::~ViewportWidget() { if (_sim) _sim->setContextHooks({}, {}); }

	void ViewportWidget::initializeGL() {
		auto* ctx = QOpenGLContext::currentContext();
		if (!ctx) {
			LOG_ERROR("No current OpenGL context");
			return;
		}

		const int gladResult = gladLoadGLLoader([](const char* name) -> void* {
			auto* ctx = QOpenGLContext::currentContext();
			if (!ctx) { return nullptr; }
			return reinterpret_cast<void*>( ctx->getProcAddress(name));
		});

		if (!gladResult) {
			LOG_ERROR("Failed to initialise GLAD");
			return;
		}

		_frameTimer.start();
		if (_sim) { 
			_sim->initGL();
			_sim->setContextHooks([this]() { makeCurrent(); }, [this]() { doneCurrent(); });
		}
	}
	void ViewportWidget::resizeGL(int w, int h) {
		if (_sim) { _sim->setDisplaySize(w, h); }
	}
	void ViewportWidget::paintGL() {
		LOG_INFO_ONCE("Qt default FBO = %u", defaultFramebufferObject());
		GLint qtFBO = defaultFramebufferObject();

		const qint64 now = _frameTimer.nsecsElapsed();
		const float dt = (_lastNs == 0) ? (1.0f / 144.0f) : static_cast<float>(now - _lastNs) * 1e-9f;
		_lastNs = now;
		if (!_sim) {
			glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			return;
		}

		_sim->setPresentationFBO(static_cast<GLuint>(qtFBO));
		_sim->tick(dt);
		_sim->renderViewport(width(), height());

		static float smoothedDt = (1.0f / 144.0f);
		smoothedDt = glm::mix(smoothedDt, dt, 0.5f);
		if (_mouseCaptured) { _sim->handleContinuousMovement(_pressedKeys, smoothedDt); }
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
				_mouseCaptured = !_mouseCaptured;
				if (_mouseCaptured) {
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
		QOpenGLWidget::keyPressEvent(event);
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
		QOpenGLWidget::keyReleaseEvent(event);
	}

	void ViewportWidget::mousePressEvent(QMouseEvent* event) {
		if (event->button() == Qt::RightButton) {
			_mouseCaptured = true;
			setFocus();
			setCursor(Qt::BlankCursor);
			QCursor::setPos(mapToGlobal(rect().center()));
			grabMouse();
			if (_sim) { _sim->resetMouseDelta(); }
		}
	}

	void ViewportWidget::mouseReleaseEvent(QMouseEvent* event) {
		if (event->button() == Qt::RightButton) {
			_mouseCaptured = false;
			releaseMouse();
			unsetCursor();
		}
	}

	void ViewportWidget::mouseMoveEvent(QMouseEvent* event) {
		if (!_sim || !_mouseCaptured) { return; }
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