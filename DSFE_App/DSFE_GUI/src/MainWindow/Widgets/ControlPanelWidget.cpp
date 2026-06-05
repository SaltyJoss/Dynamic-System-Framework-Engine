// DSFE_GUI ControlPanelWidget.cpp
#include "Widgets/ControlPanelWidget.h"
#include "Scene/SimulationManager.h"
#include "Widgets/RobotSelectorWidget.h"

#include <QScrollArea>
#include <QVBoxLayout>

namespace widgets {
	ControlPanelWidget::ControlPanelWidget(gui::SimManager* sim, QWidget* parent) : QWidget(parent), _sim(sim) {
		auto* rootLayout = new QVBoxLayout(this);
		rootLayout->setContentsMargins(4, 4, 4, 4);
		auto* scrollArea = new QScrollArea(this);
		scrollArea->setWidgetResizable(true);
		auto* content = new QWidget(scrollArea);
		auto* contentLayout = new QVBoxLayout(content);
		contentLayout->addWidget(new RobotSelectorWidget(sim, content));
		contentLayout->addStretch(); // push widgets to top	
		content->setLayout(contentLayout);
		scrollArea->setWidget(content);
		rootLayout->addWidget(scrollArea);
	}
} // namespace widgets}