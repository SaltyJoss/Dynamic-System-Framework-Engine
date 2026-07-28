
#pragma once

#include <QStyledItemDelegate>

class QPainter;
class QStyleOptionViewItem;
class QModelIndex;
class QSize;

namespace style {
    // A delegate that paints line 1 bold (title) and line 2 dim (path), Consolas.
    class RecentItemDelegate : public QStyledItemDelegate {
    public:
        using QStyledItemDelegate::QStyledItemDelegate;
        void paint(QPainter* p, const QStyleOptionViewItem& opt, const QModelIndex& idx) const override;
        QSize sizeHint(const QStyleOptionViewItem&, const QModelIndex&) const override;
    };
} // namespace style