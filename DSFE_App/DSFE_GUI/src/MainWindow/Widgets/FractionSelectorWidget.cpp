// DSFE_GUI FractionSelectorWidget.cpp
#include "Widgets/FractionSelectorWidget.h"

#include <cmath>
#include <algorithm>

#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QLabel>

namespace widgets {
	FractionSelectorWidget::FractionSelectorWidget(bool telemetryMode, QWidget* parent)
		: QWidget(parent), _telemetryMode(telemetryMode)
	{
		setMinimumSize(50, 50);
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

	void FractionSelectorWidget::setDt(double dt) {
		if (dt <= 0.0) { return; }
		const int max = _telemetryMode ? 100 : 5000;
		_k = std::clamp(static_cast<int>(std::round(1.0 / (10.0 * dt))), _minK, max);
		update();
	}

	QLabel* FractionSelectorWidget::setDtVarName(const QString& subscript) {
		auto* label = new QLabel(this);
		label->setTextFormat(Qt::RichText);
		label->setText("<span style='font-size:13pt'><i>\u0394t</i><sub>" + subscript + "</sub>\u2009=\u2009</span>");
		label->setAlignment(Qt::AlignCenter);
		return label;
	}

	QLabel* FractionSelectorWidget::setDtVarDecValue(double dt) {
		auto* label = new QLabel(this);
		label->setTextFormat(Qt::RichText);
		label->setText(QString("<span style='color:#888; font-size:12pt'>\u2192  %1</span>").arg(dt, 0, 'g', 3));
		label->setAlignment(Qt::AlignCenter);
		return label;
	}
}