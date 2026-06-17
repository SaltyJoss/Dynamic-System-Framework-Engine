// DSFE_GUI FractionSelectorWidget.cpp
#include "Widgets/FractionSelectorWidget.h"

#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>

namespace widgets {
	FractionSelectorWidget::FractionSelectorWidget(bool telemetryMode, QWidget* parent)
		: QWidget(parent), _telemetryMode(telemetryMode)
	{
		setMinimumSize(100, 70);
	}

	double FractionSelectorWidget::dt() const {
		return 1.0 / static_cast<double>(10 * _k);
	}

	void FractionSelectorWidget::paintEvent(QPaintEvent* event) {
		QPainter p(this);
		QString numer = "1";
		QString denom = QString::number(10 * _k);
		QFontMetrics fm(p.font());

		int centreX = width() / 2;
		int numerW = fm.horizontalAdvance(numer);
		int denomW = fm.horizontalAdvance(denom);

		p.drawText(centreX - numerW / 2, 20, numer);
		p.drawLine(centreX - std::max(numerW, denomW) / 2 - 2, 30, centreX + std::max(numerW, denomW) / 2 + 2, 30);
		p.drawText(centreX - denomW / 2, 50, denom);
	}

	void FractionSelectorWidget::mousePressEvent(QMouseEvent* event) { _lastMouseX = event->pos().x(); }

	void FractionSelectorWidget::mouseMoveEvent(QMouseEvent* event) {
		int dx = event->pos().x() - _lastMouseX;
		if (dx != 0) {
			_k += dx;
			int max = _telemetryMode ? 100 : 5000;
			_k = std::clamp(_k, _minK, max);
			_lastMouseX = event->pos().x();
			update();
			emit valueChanged(1.0 / static_cast<double>(10 * _k));
		}
	}

	void FractionSelectorWidget::wheelEvent(QWheelEvent* event) {
		int delta = event->angleDelta().y() > 0 ? 1 : -1;
		int max = _telemetryMode ? 100 : 5000;
		_k = std::clamp(_k + delta, _minK, max);
		update();
		emit valueChanged(1.0 / static_cast<double>(10 * _k));
	}
}