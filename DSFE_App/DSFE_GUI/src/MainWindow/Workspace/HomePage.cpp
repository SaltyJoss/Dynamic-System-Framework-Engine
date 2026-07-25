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

namespace Workspace {
    // HomePage constructor sets up the UI elements and connects signals to slots for handling user interactions.
    HomePage::HomePage(QWidget* parent) : QWidget(parent) {
        auto* root = new QHBoxLayout(this);
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

        // Right Column: Branding and primary actions (may add templates later)
        auto* right_col = new QVBoxLayout();
        right_col->setContentsMargins(0, 0, 0, 0);
        right_col->setSpacing(0);

        buildHeader(right_col);
        buildTemplateArea(right_col);
        buildDiagnosticsFooter(right_col);

        root->addWidget(rail);
        root->addLayout(right_col, 1);

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

    void HomePage::buildTemplateArea(QVBoxLayout* into) {
        auto* area = new QWidget(this);
        auto* col = new QVBoxLayout(area);
        col->setContentsMargins(40, 12, 40, 24);
        col->setSpacing(16);
        // Header for the template area
        auto* head = new QLabel("Templates", area);
        head->setObjectName("section_header");
        col->addWidget(head);
        // PLACEHOLDER CARD GRID - ill wire the templates later.
        auto* grid = new QGridLayout();
        grid->setSpacing(12);

        const char* placeholders[] = {
            "Empty Project", "Pendulum Simulation", "Spring-Mass System", "Double Pendulum",
            "Fluid Dynamics", "Electromagnetic Simulation", "VISPA Robotics Arm", "Two-Body Dynamics"
        }; // These are just cools ideas I have, may never add them, fluid and electromagnetics are probably too much for me right now, but I am dying to computational explore them
        int idx = 0;
        for (const char* name : placeholders) {
            auto* card = new QPushButton(name, area);
            card->setObjectName("template_card");
            card->setMinimumSize(180, 110);
            card->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
            card->setCursor(Qt::PointingHandCursor);
            card->setEnabled(false); // Disable for now, will enable when I wire up templates
            grid->addWidget(card, idx / 3, idx % 3);
            ++idx;
        }
        col->addLayout(grid);
        col->addStretch(1);
        into->addWidget(area, 1);
    }

    void HomePage::buildDiagnosticsFooter(QVBoxLayout* into) {
        auto* footer = new QWidget(this);
        footer->setObjectName("home_footer");
        auto* row = new QHBoxLayout(footer);
        row->setContentsMargins(40, 12, 40, 12);

        // Placeholder for diagnostics information, will wire up after checking build
        auto* diag = new QLabel("System diagnostics loading...", footer);
        diag->setObjectName("footer_text");
        row->addWidget(diag);
        row->addStretch(1);

        auto* version = new QLabel("DSFE v1.0.0-beta", footer);
        version->setObjectName("footer_version");
        row->addWidget(version);
        row->addSpacing(20);

        into->addWidget(footer);
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