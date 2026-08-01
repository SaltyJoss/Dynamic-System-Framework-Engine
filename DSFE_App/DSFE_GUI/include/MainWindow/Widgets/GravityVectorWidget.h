/*
 * Project: DSFE_GUI
 * File: MainWindow/Widgets/GravityVectorWidget.h
 * Created by: Joss Salton, 27-07-2026
 */
#pragma once

#include <QWidget>
#include <functional>
#include <glm/glm.hpp>

class QDoubleSpinBox;
class QLabel;

namespace widgets {
    class GravityVectorWidget : public QWidget {
        Q_OBJECT
        public:
            explicit GravityVectorWidget(QWidget* parent = nullptr);
            void setValue(const glm::vec3& g);
            glm::vec3 value() const;
            std::function<void(const glm::vec3&)> onChanged;

        private:
            void emitChanged();
            void refreshLabel();
            QLabel* _tex;
            QDoubleSpinBox* _x; QDoubleSpinBox* _y; QDoubleSpinBox* _z;
    };
} // namespace widgets