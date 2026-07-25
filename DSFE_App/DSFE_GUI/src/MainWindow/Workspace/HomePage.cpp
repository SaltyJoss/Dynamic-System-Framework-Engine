// DSFE_GUI HomePage.cpp
#include "Workspace/HomePage.h"
#include "Workspace/RecentWorkspace.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
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
        auto* left = new QWidget(this);
        auto* left_col = new QVBoxLayout(left);
        left_col->setContentsMargins(32,48,32,48);

        auto* recents_head = new QLabel("Recent Projects", left);
        QFont rh = recents_head->font();
        rh.setPointSize(14); rh.setBold(true);
        recents_head->setFont(rh);
        left_col->addWidget(recents_head);
        left_col->addSpacing(16);

        _recents_list = new QListWidget(left);
        _recents_list->setFrameShape(QFrame::NoFrame);
        left_col->addWidget(_recents_list, 1);

        // Right Column: Branding and primary actions (may add templates later)
        auto* right = new QWidget(this);
        auto* right_col = new QVBoxLayout(right);
        right_col->setContentsMargins(48,48,48,48);
        right_col->addStretch(2);

        auto* title = new QLabel("DSFE", right);
        QFont t = title->font();
        t.setPointSize(48); t.setBold(true);
        title->setFont(t);
        right_col->addWidget(title);

        auto* subtitle = new QLabel("Dynamic System Framework Engine", right);
        QFont st = subtitle->font();
        st.setPointSize(14);
        subtitle->setFont(st);
        subtitle->setStyleSheet("color: rgb(140,140,140);");
        right_col->addWidget(subtitle);

        right_col->addSpacing(32);

        auto* new_btn = new QPushButton("New Project", right);
        auto* open_btn = new QPushButton("Open Project", right);
        for (auto* btn : {new_btn, open_btn}) {
            btn->setMinimumHeight(48);
            btn->setMaximumWidth(280);
            btn->setCursor(Qt::PointingHandCursor);
        }
        right_col->addWidget(new_btn);
        right_col->addSpacing(16);
        right_col->addWidget(open_btn);

        right_col->addStretch(3);

        auto* version = new QLabel("v1.0.0-beta", right);
        version->setStyleSheet("color: rgb(90,90,90);");
        right_col->addWidget(version, 0, Qt::AlignRight);

        // Assemble root layout
        root->addWidget(left, 1);  // Left column:  25%
        root->addWidget(right, 3); // Right column: 75%

        connect(new_btn, &QPushButton::clicked, this, [this]() { if (onNewProject) { onNewProject(); } });
        connect(open_btn, &QPushButton::clicked, this, [this]() { if (onOpenProject) { onOpenProject(); } });
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