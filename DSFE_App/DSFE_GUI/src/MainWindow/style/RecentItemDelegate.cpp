
#include "style/RecentItemDelegate.h"

#include <QPainter>
#include <QStyleOptionViewItem>

namespace style {
    // A delegate that paints line 1 bold (title) and line 2 dim (path), Consolas.
    void RecentItemDelegate::paint(QPainter* p, const QStyleOptionViewItem& opt, const QModelIndex& idx) const {
        // selection/hover background — let the base draw it so your sheet's blue applies
        QStyleOptionViewItem o(opt);
        initStyleOption(&o, idx);
        o.text.clear();
        o.widget->style()->drawControl(QStyle::CE_ItemViewItem, &o, p, o.widget);

        const QString full = idx.data(Qt::DisplayRole).toString();
        const int nl = full.indexOf('\n');
        const QString title = nl >= 0 ? full.left(nl) : full;
        const QString sub   = nl >= 0 ? full.mid(nl + 1) : QString();

        QRect r = opt.rect.adjusted(8, 4, -8, -4);
        const bool sel = opt.state & QStyle::State_Selected;

        QFont tf("Consolas"); tf.setBold(true); tf.setPointSize(10);
        p->setFont(tf);
        p->setPen(sel ? QColor(255,255,255) : QColor(230,230,235));
        p->drawText(QRect(r.x(), r.y(), r.width(), r.height()/2),
            Qt::AlignLeft | Qt::AlignVCenter, title);

        QFont sf("Consolas"); sf.setPointSize(8);
        p->setFont(sf);
        p->setPen(sel ? QColor(220,225,235) : QColor(140,140,148));
        p->drawText(QRect(r.x(), r.y() + r.height()/2, r.width(), r.height()/2),
            Qt::AlignLeft | Qt::AlignVCenter,
            p->fontMetrics().elidedText(sub, Qt::ElideMiddle, r.width()));
    }
    // sizeHint is the height of two lines of Consolas, plus 8px vertical padding.
    QSize RecentItemDelegate::sizeHint(const QStyleOptionViewItem&, const QModelIndex&) const { return QSize(0, 46); }
} // namespace style