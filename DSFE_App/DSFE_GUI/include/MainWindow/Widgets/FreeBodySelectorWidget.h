/*
 * File: DSFE_GUI/include/MainWindow/Widgets/FreeBodySelectorWidget.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include <QWidget>

namespace gui { class SimulationManager; }
class QPushButton;
namespace widgets {
    class FreeBodySelectorWidget : public QWidget {
        public:
            explicit FreeBodySelectorWidget(gui::SimulationManager* sim, QWidget* parent = nullptr);
        private:
            gui::SimulationManager* _sim;
            void addFreeBodyButton(const QString& bodyName, QString company);
    };
}