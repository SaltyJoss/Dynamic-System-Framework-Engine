// DSFE_GUI HomePage.cpp
#include "Workspace/HomePage.h"
#include "Workspace/RecentWorkspace.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QListWidget>
#include <QFileInfo>
#include <QMenu>
#include <QFrame>
#include <QPixmap>
#include <QMouseEvent>
#include <functional>

#include "Platform/SystemInfo.h"

#include "Platform/Paths.h"
#include "EngineLib/LogMacros.h"

namespace {
    // Click template card: Thumbnail on top, label beneath. For now, just a placeholder with a colored background and a label.
    class TemplateCard : public QWidget {
        public:
            TemplateCard(const QString& name, const QString& image_path, QWidget* parent = nullptr) 
                : QWidget(parent) 
            {
                setObjectName("template_card");
                setCursor(Qt::PointingHandCursor);
                auto* col = new QVBoxLayout(this);
                col->setContentsMargins(0, 0, 0, 0);
                col->setSpacing(0);
                _thumb = new QLabel(this);
                _thumb->setObjectName("card_thumb");
                _thumb->setAlignment(Qt::AlignCenter);
                _thumb->setFixedHeight(150);
                QPixmap pm(image_path);
                if (!pm.isNull()) {
                    _thumb->setPixmap(pm.scaled(300, 150, Qt::KeepAspectRatioByExpanding, Qt::SmoothTransformation));
                } else {
                    _thumb->setText("no preview");
                }
                _thumb->setScaledContents(false);
                col->addWidget(_thumb);
                auto* caption = new QLabel(name, this);
                caption->setObjectName("card_caption");
                caption->setAlignment(Qt::AlignCenter);
                caption->setFixedHeight(38);
                col->addWidget(caption);
            }  

            std::function<void()> onClick; // Callback for when the card is clicked
        protected:
            void mousePressEvent(QMouseEvent* event) override { if (onClick) { onClick(); } }

        private:
            QLabel* _thumb = nullptr;
    };
}

namespace Workspace {
    // HomePage constructor sets up the UI elements and connects signals to slots for handling user interactions.
    HomePage::HomePage(QWidget* parent) : QWidget(parent) {
        auto* root = new QVBoxLayout(this);
        root->setContentsMargins(0, 0, 0, 0);
        root->setSpacing(0);

        // Left Column - Recents List, fills height 
        auto* rail = new QWidget(this);
        rail->setObjectName("home_rail");
        rail->setFixedWidth(340);
        auto* rail_col = new QVBoxLayout(rail);
        rail_col->setContentsMargins(20, 24, 20, 24);
        rail_col->addSpacing(12);

        auto* recents_head = new QLabel("Recent Projects", rail);
        recents_head->setObjectName("rail_header");
        rail_col->addWidget(recents_head);
        
        _recents_list = new QListWidget(rail);
        _recents_list->setFrameShape(QFrame::NoFrame);
        rail_col->addWidget(_recents_list, 1);

        // Right Column: Branding and primary actions
        auto* right_col = new QVBoxLayout();
        right_col->setContentsMargins(0, 0, 0, 0);
        right_col->setSpacing(0);

        buildHeader(right_col);

        auto* main_row = new QHBoxLayout();
        main_row->setContentsMargins(0, 0, 0, 0);
        main_row->setSpacing(0);
        buildTemplatesArea(main_row);
        buildDiagnosticsPanel(main_row);
        right_col->addLayout(main_row, 1);

        // Combine the left and right columns into the main layout
        auto* split_row = new QHBoxLayout();
        split_row->setContentsMargins(0, 0, 0, 0);
        split_row->setSpacing(0);
        split_row->addWidget(rail);
        split_row->addLayout(right_col, 1);
        root->addLayout(split_row, 1);

        // Slim version footer spanning the bottom of the content column.
        auto* footer = new QWidget(this);
        footer->setObjectName("home_footer");
        footer->setFixedHeight(30);
        auto* footer_row = new QHBoxLayout(footer);
        footer_row->setContentsMargins(40, 0, 40, 0);
        footer_row->addStretch(1);
        auto* version = new QLabel("DSFE v1.0.0-beta", footer);
        version->setObjectName("footer_text");
        footer_row->addWidget(version);
        root->addWidget(footer);

        connect(_recents_list, &QListWidget::itemDoubleClicked, this, [this](QListWidgetItem* item) {
            const QString path = item->data(Qt::UserRole).toString();
            if (!path.isEmpty() && onOpenRecent) { onOpenRecent(path); }
        });

        _recents_list->setContextMenuPolicy(Qt::CustomContextMenu);
        connect(_recents_list, &QListWidget::customContextMenuRequested, this, [this](const QPoint& pos) {
            QListWidgetItem* item = _recents_list->itemAt(pos);
            if (!item) { return; }
            const QString path = item->data(Qt::UserRole).toString();
            if (path.isEmpty()) { return; }
            QMenu menu(this);
            QAction* open   = menu.addAction("Open");
            QAction* remove = menu.addAction("Remove from list");
            QAction* chosen = menu.exec(_recents_list->mapToGlobal(pos));
            if (chosen == open && onOpenRecent) { onOpenRecent(path); }
            else if (chosen == remove) { gui::RecentWorkspaces::remove(path); refreshRecents(); }
        });

        refreshRecents();
    }

    void HomePage::buildHeader(QVBoxLayout* into) {
        auto* head = new QWidget(this);
        head->setObjectName("home_header");
        auto* row = new QHBoxLayout(head);
        row->setContentsMargins(40, 12, 40, 12);
        // Branding block, left-aligned
        auto* brand = new QVBoxLayout();
        brand->setSpacing(2);
        auto* title = new QLabel("DSFE", head);
        title->setObjectName("brand_title");
        auto* subtitle = new QLabel("Dynamic System Framework Engine", head);
        subtitle->setObjectName("brand_subtitle");
        brand->addWidget(title);
        brand->addWidget(subtitle);
        // Add the branding and primary action buttons to the header row
        row->addLayout(brand);
        row->addStretch(1);
        // Primary Actions
        auto* new_btn = new QPushButton("New Project", head);
        auto* open_btn = new QPushButton("Open Project", head);
        new_btn->setObjectName("prim_btn");
        open_btn->setObjectName("second_btn");
        for (QPushButton* b : { new_btn, open_btn }) {
            b->setMinimumHeight(38);
            b->setMaximumWidth(130);
            b->setCursor(Qt::PointingHandCursor);
        }
        // Add the buttons to the header row with spacing
        row->addWidget(new_btn);
        row->addSpacing(12);
        row->addWidget(open_btn);

        // Connect button clicks to the corresponding callbacks if they are set
        connect(new_btn, &QPushButton::clicked, this, [this]() { if (onNewProject) { onNewProject(); } });
        connect(open_btn, &QPushButton::clicked, this, [this]() { if (onOpenProject) { onOpenProject(); } });
        into->addWidget(head);
    }

    void HomePage::buildTemplatesArea(QHBoxLayout* into) {
        auto* area = new QWidget(this);
        auto* col = new QVBoxLayout(area);
        col->setContentsMargins(40, 12, 24, 24);
        col->setSpacing(16);
        // Header for the template area
        auto* head = new QLabel("Templates", area);
        head->setObjectName("section_header");
        col->addWidget(head);
        // PLACEHOLDER CARD GRID - ill wire the templates later.
        auto* grid = new QGridLayout();
        grid->setSpacing(12);

        struct TplDef { const char* name; const char* image; };
        const TplDef templates[] = {
            { "Empty Project",       "empty.png" },
            { "Pendulum Simulation", "pendulum.png" },
            { "Spring-Mass System",  "spring_mass.png" },
            { "Double Pendulum",     "double_pendulum.png" },
            { "VISPA Robotics Arm",  "vispa.png" },
            { "Two-Body Dynamics",   "two_body.png" },
        };
        const QString imgDir = QString::fromStdString((paths::assets() / "templates" / "thumbnails").string());
        int idx=0;
        for (const TplDef& tpl : templates) {
            auto* card = new TemplateCard(tpl.name, imgDir + "/" + tpl.image, area);
            card->setFixedWidth(300);
            card->onClick = [this, tpl]() { LOG_INFO("Template clicked: %s", tpl.name); };
            grid->addWidget(card, idx/3, idx%3);
            ++idx;
        }
        grid->setColumnStretch(3, 1); // Add stretch to the last column to push cards to the left
        col->addLayout(grid);
        col->addStretch(1);
        into->addWidget(area, 3);   // templates take ~75% of the main row
    }

void HomePage::buildDiagnosticsPanel(QHBoxLayout* into) {
        auto* panel = new QWidget(this);
        panel->setObjectName("diag_panel");
        auto* col = new QVBoxLayout(panel);
        col->setContentsMargins(24, 12, 40, 24);
        col->setSpacing(12);

        auto* header = new QLabel("System Information:", panel);
        header->setObjectName("section_header");
        header->setStyleSheet("font-size: 15px; font-weight: 600; ");
        col->addWidget(header);

        _diag_content = new QWidget(panel);
        _diag_content->setAttribute(Qt::WA_TranslucentBackground);
        auto* placeholder_col = new QVBoxLayout(_diag_content);
        placeholder_col->setContentsMargins(0, 0, 0, 0);
        auto* loading = new QLabel("Loading system information...", _diag_content);
        loading->setObjectName("footer_text");
        placeholder_col->addWidget(loading);
        col->addWidget(_diag_content);

        col->addStretch(1);

        into->addWidget(panel, 1);   // diagnostics take ~25% of the main row

        populateDiagnostics();
    }

    void HomePage::populateDiagnostics() {
        if (!_diag_content) { return; }
        QLayout* old = _diag_content->layout();
        if (old) { 
            QLayoutItem* item;
            while ((item = old->takeAt(0)) != nullptr) {
                if (item->widget()) { item->widget()->deleteLater(); }
                delete item;
            } 
            delete old; 
        }
        const gui::SystemInfo si = gui::SystemInfo::query();
        auto* row = new QVBoxLayout(_diag_content);
        row->setContentsMargins(0, 0, 0, 0);
        row->setSpacing(16);

        // Helper: coloured key chip + value, neofetch-style
        auto addField = [&](const QString& key, const QString& value, const QString& keyColour) {
            auto* cell = new QWidget(_diag_content);
            cell->setStyleSheet("background: transparent;");
            auto* h = new QHBoxLayout(cell);
            h->setContentsMargins(0, 0, 0, 0);
            h->setSpacing(6);
            auto* dot = new QLabel("  ●", cell);
            dot->setStyleSheet(QString("color: %1; font-size: 10px; background: transparent;").arg(keyColour));
            auto* k = new QLabel(key, cell);
            k->setStyleSheet(QString("color: %1; font-weight: 600; background: transparent;").arg(keyColour));
            k->setFixedWidth(64);
            auto* v = new QLabel(value, cell);
            v->setObjectName("footer_text");
            h->addWidget(dot);
            h->addWidget(k);
            h->addWidget(v);
            h->addStretch(1);
            row->addWidget(cell);
        };
        // neofetch-ish palette (because I like it)
        addField("CPU", si.processor, "rgb(102,187,106)");   // green
        addField("RAM", si.ram, "rgb(255,167,38)");          // amber
        addField("GPU", si.gpus.join("  |  "), "rgb(86,156,214)");  // blue accent
        addField("OS", si.os, "rgb(236,64,122)");     // pink
        if (!si.vulkanVersion.isEmpty()) { addField("VK", si.vulkanVersion, "rgb(120,144,156)"); }  // slate
        // Live solver status - green if a Vulkan device was found.
        const bool ready = !si.gpus.isEmpty() && si.gpus.first() != "No hardware GPU";
        addField("SOLVER", ready ? "Ready" : "CPU only", ready ? "rgb(102,187,106)" : "rgb(255, 8, 0)");
    }

    // refreshRecents repopulates the recent projects list in the UI based on the stored recent workspaces.
    void HomePage::refreshRecents() {
        _recents_list->clear();
        const QStringList recents = gui::RecentWorkspaces::list();
        for (const QString& path : recents) {
            auto* item = new QListWidgetItem();
            item->setData(Qt::UserRole, path);
            item->setText(QFileInfo(path).baseName() + "\n" + path);
            item->setData(Qt::ToolTipRole, path);
            _recents_list->addItem(item);
        }
        if (recents.empty()) {
            auto* item = new QListWidgetItem("No recent projects.\nCreate or open one to get started.");
            item->setFlags(Qt::NoItemFlags);
            _recents_list->addItem(item);
        }
    }
} // namespace Workspace