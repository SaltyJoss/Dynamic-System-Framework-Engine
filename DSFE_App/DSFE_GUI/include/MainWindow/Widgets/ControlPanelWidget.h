// DSFE_GUI ControlPanelWidget.h
#pragma once

#include <QWidget>

namespace gui { class SimManager; }

namespace widgets {
	class ControlPanelWidget : public QWidget {
	public:
		explicit ControlPanelWidget(gui::SimManager* sim, QWidget* parent = nullptr);
	private:
		gui::SimManager* _sim = nullptr;
	};
} // namespace widgets