/*
 * File: DSFE_GUI/src/MainWindow/Widgets/FreeBodySelectorWidget.cpp
 * Created by: Joss Salton, 26-07-2026
 */
#include "Widgets/FreeBodySelectorWidget.h"
#include "Simulation/SimulationManager.h"

#include "Platform/SystemMap.h"

#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>

namespace widgets {
    FreeBodySelectorWidget::FreeBodySelectorWidget(gui::SimulationManager* sim, QWidget* parent) : QWidget(parent), _sim(sim) {
        setWindowTitle("Choose Free-Body");
        setMinimumWidth(300);
        auto* layout = new QVBoxLayout(this);
        if (!_sim) {
            layout->addWidget(new QLabel("No simulation manager available", this));
            return;
        }

        const std::unordered_map<platform::eFreeBodies, platform::eFreeBodyFamilies>& bodyMap = platform::getFreeBodySystemMap();

        if (bodyMap.empty()) {
            layout->addWidget(new QLabel("No free-bodies available", this));
            return;
        }

        for (const auto& [sys, family] : bodyMap) {
            const QString bodyName = QString::fromStdString(platform::FreeBodies().toString(sys));
            const QString familyName = QString::fromStdString(platform::FreeBodies().toString(family));
            addFreeBodyButton(bodyName, familyName);
        }
    }

    void FreeBodySelectorWidget::addFreeBodyButton(const QString& bodyName, QString company) {
        auto* button = new QPushButton(bodyName + "\n" + company);
        layout()->addWidget(button);
        connect(button, &QPushButton::clicked, this, [this, bodyName]() {
            std::string n = bodyName.toStdString();
            std::transform(n.begin(), n.end(), n.begin(), [](unsigned char c){ return std::tolower(c); });
            const std::string path = "rigidbody_models/" + n + "/" + n + ".urdf";
            LOG_INFO("Selected free body: %s -> %s", n.c_str(), path.c_str());
            if (_sim) { _sim->load_rigidBody(path); }
        });
    }
}