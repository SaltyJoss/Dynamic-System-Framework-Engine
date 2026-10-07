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
		_sim->initialiseRenderer(n_win, static_cast<uint32_t>(width()), static_cast<uint32_t>(height()));
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
		if (_sim && _renderer_initialised) { _sim->resizeRenderer(static_cast<uint32_t>(width()), static_cast<uint32_t>(height())); }
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
		else if (event->button() == Qt::LeftButton && !_mouse_captured && _sim) {
			// Pick the link whose world origin is nearest the cursor ray.
			_dragLink = pickLink(event->position().x(), event->position().y());
			if (!_dragLink.empty()) { 
				_dragging = true;
				_sim->setManipulating(true);
				_sim->setLinkHighlight(_dragLink, true);
			}
		}
	}

	void ViewportWidget::mouseReleaseEvent(QMouseEvent* event) {
		if (event->button() == Qt::RightButton) {
			_mouse_captured = false;
			releaseMouse();
			unsetCursor();
		}
		else if (event->button() == Qt::LeftButton && _dragging) {
			_dragging = false;
			if (_sim && !_dragLink.empty()) { _sim->setLinkHighlight(_dragLink, false); }
			_dragLink.clear();
			if (_sim) { _sim->clearExternalForces(); }
		}
	}

	void ViewportWidget::mouseMoveEvent(QMouseEvent* event) {
		if (!_sim) { return; }
		if (_mouse_captured) {
			QPoint current = QCursor::pos();
			QPoint delta = current - _screenCenter;
			_sim->handleMouseLook(delta.x(), -delta.y(), true);
			QCursor::setPos(_screenCenter);
			return;
		}
		// If dragging a link, apply a spring-damper force to move the link toward the cursor's projected position.
		if (_dragging && !_dragLink.empty()) {
			// Unproject the cursor onto a view-parallel plane through the grab point, then push the link toward it with a spring.
			glm::vec3 target = cursorToDragPlane(event->position().x(), event->position().y());
			if (target.y < 0.0f) { target.y = 0.0f; }   // never drag a link below the floor plane
			const glm::vec3 grab = linkOrigin(_dragLink);
			// Approximation of the grabbed points velocity from its moption since the last frame, to reduce spring oscillation (Massive issue with freebodies)
			static glm::vec3 s_lastGrab = grab;
			static auto s_lastTime = std::chrono::high_resolution_clock::now();
			const auto now = std::chrono::high_resolution_clock::now();
			const float dt = std::max(1e-4f, std::chrono::duration<float>(now - s_lastTime).count());
			const glm::vec3 grabVel = (grab - s_lastGrab) / dt;
			s_lastGrab = grab; s_lastTime = now;
			// Spring-damper force: F = k * (target - grab) - c * v, where v is the velocity of the grabbed point. Limit the force magnitude to avoid instability.
			const float k = 80.0f; // Stiffness of the spring (N/m)
			const float c = 40.0f;  // Damping coefficient (N·s/m)
			glm::vec3 force = k * (target - grab) - c * grabVel;
			// Limit the force magnitude to avoid instability
			const float f_max = 400.0f; // Maximum force magnitude (N) -> My previous was 3000, so I will likely change this as I test the new collision system
			if (glm::length(force) > f_max) { force = glm::normalize(force) * f_max; }
			_sim->setLinkExternalForce(_dragLink, grab, force);
		}
	}

	void ViewportWidget::wheelEvent(QWheelEvent* event) {
		if (!_sim) { return; }
		_sim->onMouseWheel(event->angleDelta().y() / 120.0);
	}

	glm::vec3 ViewportWidget::linkOrigin(const std::string& name) const {
		const auto& xf = _sim->linkWorldTransforms();
		const auto  names = _sim->linkNames();
		for (size_t i = 0; i < names.size() && i < xf.size(); ++i) {
			if (names[i] == name) {
				const mathlib::Mat4& m = xf[i];
				return glm::vec3((float)m(0,3), (float)m(1,3), (float)m(2,3));
			}
		}
		return glm::vec3(0.0f);
	}

	// Build a world-space ray from the cursor, return the nearest link name.
	std::string ViewportWidget::pickLink(float sx, float sy) const {
		scene::Camera& cam = _sim->camera();
		const glm::mat4 invVP = glm::inverse(cam.getProjection() * cam.getViewMatrix());
		// NDC (Vulkan Y-down: flip y). Near/far points -> ray.
		const float ndcX =  2.0f * (sx / (float)width()) - 1.0f;
		const float ndcY =  2.0f * (sy / (float)height()) - 1.0f; // no extra flip: screen y-down matches
		glm::vec4 pNear = invVP * glm::vec4(ndcX, ndcY, 0.0f, 1.0f);
		glm::vec4 pFar  = invVP * glm::vec4(ndcX, ndcY, 1.0f, 1.0f);
		pNear /= pNear.w; pFar /= pFar.w;
		const glm::vec3 o(pNear);
		const glm::vec3 d = glm::normalize(glm::vec3(pFar) - glm::vec3(pNear));
		const auto& xf = _sim->linkWorldTransforms();
		const auto  names = _sim->linkNames();
		std::string best; float bestDist = 1e9f;
		for (size_t i = 0; i < names.size() && i < xf.size(); ++i) {
			const glm::vec3 p((float)xf[i](0,3), (float)xf[i](1,3), (float)xf[i](2,3));
			// distance from link origin to the ray
			const float t = glm::dot(p - o, d);
			if (t < 0.0f) { continue; }
			const float perp = glm::length((o + d * t) - p);
			if (perp < bestDist && perp < 0.5f) { bestDist = perp; best = names[i]; } // 0.5m pick radius
		}
		return best;
	}

	// Project the cursor onto a plane through the grab point, normal = camera forward.
	glm::vec3 ViewportWidget::cursorToDragPlane(float sx, float sy) const {
		scene::Camera& cam = _sim->camera();
		const glm::mat4 invVP = glm::inverse(cam.getProjection() * cam.getViewMatrix());
		const float ndcX = 2.0f * (sx / (float)width()) - 1.0f;
		const float ndcY = 2.0f * (sy / (float)height()) - 1.0f;
		glm::vec4 pNear = invVP * glm::vec4(ndcX, ndcY, 0.0f, 1.0f);
		glm::vec4 pFar  = invVP * glm::vec4(ndcX, ndcY, 1.0f, 1.0f);
		pNear /= pNear.w; pFar /= pFar.w;
		const glm::vec3 o(pNear);
		const glm::vec3 d = glm::normalize(glm::vec3(pFar) - glm::vec3(pNear));
		const glm::vec3 planePt = linkOrigin(_dragLink);
		const glm::vec3 n = -cam.getForward(); // plane faces the camera
		const float denom = glm::dot(d, n);
		if (std::abs(denom) < 1e-6f) { return planePt; }
		const float t = glm::dot(planePt - o, n) / denom;
		return o + d * t;
	}

} // namespace widgets