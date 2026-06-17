// DSFE_GUI DSLSyntaxHighlighter.h
#pragma once

#include <QSyntaxHighlighter>
#include <QRegularExpression>

class QColor;

namespace widgets {

	class DSLSyntaxHighlighter : public QSyntaxHighlighter {
	public:
		explicit DSLSyntaxHighlighter(QTextDocument* parent = nullptr);

	protected:
		void highlightBlock(const QString& text) override;

	private:
		template <typename Container>
		QColor makeQColor(const Container& c) {
			if (c.size() < 3) return QColor(); // Returns an invalid color safely
			return QColor(c[0], c[1], c[2]);
		}

		struct HighlightingRule {
			QRegularExpression pattern;
			QTextCharFormat format;
		};
		std::vector<HighlightingRule> _rules;
	};

}