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

namespace Workspace {
    // HomePage constructor sets up the UI elements and connects signals to slots for handling user interactions.
    HomePage::HomePage(QWidget* parent) : QWidget(parent) {
        auto* outer = new QVBoxLayout(this);
        outer->addStretch(2);

        auto* title = new QLabel("DSFE", this);
        QFont tf = title->font(); tf.setPointSize(36); tf.setBold(true);
        title->setFont(tf);
        title->setAlignment(Qt::AlignCenter);
        outer->addWidget(title);

        auto* subtitle = new QLabel("Dynamic System Framework Engine", this);
        subtitle->setAlignment(Qt::AlignCenter);
        outer->addWidget(subtitle);

        outer->addSpacing(24);

        auto* buttonRow = new QHBoxLayout();
        buttonRow->addStretch(1);
        auto* newBtn = new QPushButton("New Project", this);
        auto* openBtn = new QPushButton("Open Project…", this);
        newBtn->setMinimumSize(160, 40);
        openBtn->setMinimumSize(160, 40);
        buttonRow->addWidget(newBtn);
        buttonRow->addSpacing(12);
        buttonRow->addWidget(openBtn);
        buttonRow->addStretch(1);
        outer->addLayout(buttonRow);

        outer->addSpacing(24);

        auto* recentsLabel = new QLabel("Recent Projects", this);
        recentsLabel->setAlignment(Qt::AlignCenter);
        outer->addWidget(recentsLabel);

        _recentsList = new QListWidget(this);
        _recentsList->setMaximumWidth(560);
        _recentsList->setMaximumHeight(220);
        auto* listRow = new QHBoxLayout();
        listRow->addStretch(1);
        listRow->addWidget(_recentsList);
        listRow->addStretch(1);
        outer->addLayout(listRow);

        outer->addStretch(3);

        connect(newBtn, &QPushButton::clicked, this, [this]() { if (onNewProject) { onNewProject(); } });
        connect(openBtn, &QPushButton::clicked, this, [this]() { if (onOpenProject) { onOpenProject(); } });
        connect(_recentsList, &QListWidget::itemDoubleClicked, this, [this](QListWidgetItem* item) {
            const QString path = item->data(Qt::UserRole).toString();
            if (!path.isEmpty() && onOpenRecent) { onOpenRecent(path); }
        });

        _recentsList->setContextMenuPolicy(Qt::CustomContextMenu);
        connect(_recentsList, &QListWidget::customContextMenuRequested, this, [this](const QPoint& pos) {
            QListWidgetItem* item = _recentsList->itemAt(pos);
            if (!item) { return; }
            const QString path = item->data(Qt::UserRole).toString();
            if (path.isEmpty()) { return; }

            QMenu menu(this);
            QAction* open   = menu.addAction("Open");
            QAction* remove = menu.addAction("Remove from list");
            QAction* chosen = menu.exec(_recentsList->mapToGlobal(pos));
            if (chosen == open && onOpenRecent) { onOpenRecent(path); }
            else if (chosen == remove) {
                gui::RecentWorkspaces::remove(path);
                refreshRecents();
            }
        });

        refreshRecents();
    }

    // refreshRecents repopulates the recent projects list in the UI based on the stored recent workspaces.
    void HomePage::refreshRecents() {
        _recentsList->clear();
        for (const QString& path : gui::RecentWorkspaces::list()) {
            auto* item = new QListWidgetItem(QFileInfo(path).baseName() + "    -    " + path);
            item->setData(Qt::UserRole, path);
            _recentsList->addItem(item);
        }
        if (_recentsList->count() == 0) {
            auto* item = new QListWidgetItem("(no recent projects)");
            item->setFlags(Qt::NoItemFlags);
            _recentsList->addItem(item);
        }
    }
} // namespace Workspace