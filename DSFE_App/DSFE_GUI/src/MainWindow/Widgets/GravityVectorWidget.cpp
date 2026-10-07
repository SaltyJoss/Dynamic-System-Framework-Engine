/*
 * Project: DSFE_GUI
 * File: MainWindow/Widgets/GravityVectorWidget.cpp
 * Created by: Joss Salton, 27-07-2026
 */
#include "Widgets/GravityVectorWidget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDoubleSpinBox>
#include <QLabel>

namespace widgets {
    static QDoubleSpinBox* makeBox() {
        auto* b = new QDoubleSpinBox();
        b->setRange(-100.0, 100.0); // Set a reasonable range for gravity values
        b->setDecimals(3);
        b->setSingleStep(0.1);
        b->setMinimumWidth(25);
        b->setSuffix(" m/s\u00B2"); // Unicode for squared symbol
        return b;
    }

    GravityVectorWidget::GravityVectorWidget(QWidget* parent) : QWidget(parent) {
        _tex = new QLabel(this);
        _tex->setTextFormat(Qt::RichText);
        _tex->setAlignment(Qt::AlignCenter);
        _x = makeBox(); _y = makeBox(); _z = makeBox();

        auto* row = new QHBoxLayout();
        auto add_labelled = [&](const char* axis, QDoubleSpinBox* b) {
            auto* col = new QVBoxLayout();
            auto* l = new QLabel(QString("<b><i>%1</i></b>").arg(axis));
            l->setAlignment(Qt::AlignCenter);
            col->addWidget(l); col->addWidget(b);
            row->addLayout(col);
        };
        add_labelled("x", _x); add_labelled("y", _y); add_labelled("z", _z);

        auto* root = new QVBoxLayout(this);
        root->addWidget(_tex);
        root->addLayout(row);

        for (auto* b : {_x, _y, _z}) {
            connect(b, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this]() { refreshLabel(); emitChanged(); });
        }
        refreshLabel();
    }

    void GravityVectorWidget::refreshLabel() {
        _tex->setText(
            QString(
                "<span style='font-size:13pt'>"
                "<b><i>g</i></b>\u2009=\u2009[ %1, %2, %3 ]<sup>T</sup> "
                "<span style='color:#888'>m/s<sup>2</sup></span></span>"
            )
            .arg(_x->value(), 0, 'f', 3)
            .arg(_y->value(), 0, 'f', 3)
            .arg(_z->value(), 0, 'f', 3)
        );
    }

    void GravityVectorWidget::setValue(const glm::vec3& g) {
        QSignalBlocker bx(_x), by(_y), bz(_z);
        _x->setValue(g.x); _y->setValue(g.y); _z->setValue(g.z);
        refreshLabel();
    }

    glm::vec3 GravityVectorWidget::value() const {
        return glm::vec3(static_cast<float>(_x->value()), static_cast<float>(_y->value()), static_cast<float>(_z->value()));
    }

    void GravityVectorWidget::emitChanged() {
        if (onChanged) { onChanged(value()); }
    }

    
} // namespace widgets