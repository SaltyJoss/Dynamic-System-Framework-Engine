// DSFE_GUI FractionSelectorWidget.h
#pragma once

#include <QWidget>

namespace widgets {
	class FractionSelectorWidget : public QWidget {
	public:
		explicit FractionSelectorWidget(bool telemetryMode = false, QWidget* parent = nullptr);
		double dt() const;
	
	signals:
		void valueChanged(double denom);

	protected:
		void paintEvent(QPaintEvent* event) override;
		void mousePressEvent(QMouseEvent* event) override;
		void mouseMoveEvent(QMouseEvent* event) override;
		void wheelEvent(QWheelEvent* event) override;
	
	private:
		int _k = 6;
		int _minK = 1;
		int _lastMouseX = 0;
		bool _telemetryMode = false; // If true, the widget is being used to select a telemetry recording fraction, so it should display "Telemetry: 1/k" instead of "dt: 1/k"
	};
}