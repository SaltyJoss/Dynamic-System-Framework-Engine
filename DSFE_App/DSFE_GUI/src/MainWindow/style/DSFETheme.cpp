// DSFE_GUI DSFETheme.cpp
#include "DSFETheme.h"

#include <QApplication>
#include <QPalette>
#include <QFile>
#include <QColor>


namespace style {
	void applyTheme(QApplication& app) {
		QPalette palette;
		palette.setColor(QPalette::Window, QColor(30, 30, 30));
		palette.setColor(QPalette::WindowText, QColor(220, 220, 220));
		palette.setColor(QPalette::Base, QColor(30, 30, 30));
		palette.setColor(QPalette::AlternateBase, QColor(40, 40, 40));
		palette.setColor(QPalette::Text, QColor(220, 220, 220));
		palette.setColor(QPalette::Button, QColor(40, 40, 40));
		palette.setColor(QPalette::ButtonText, QColor(220, 220, 220));
		palette.setColor(QPalette::Highlight, QColor(86, 156, 214));
		palette.setColor(QPalette::HighlightedText, QColor(255, 255, 255));
		app.setPalette(palette);

		QFile file(":/style/dsfe_dark.qss");
		if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) { return; }

		app.setStyleSheet(QString::fromUtf8(file.readAll()));
	}

}