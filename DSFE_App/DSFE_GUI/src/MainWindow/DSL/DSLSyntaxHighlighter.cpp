// DSFE_GUI DSLSyntaxHighlighter.cpp
#include "DSL/DSLSyntaxHighlighter.h"
#include "DSL/DSLSyntaxColours.h"

namespace widgets {
	DSLSyntaxHighlighter::DSLSyntaxHighlighter(QTextDocument* parent) : QSyntaxHighlighter(parent) {
		QTextCharFormat commentFmt;
		commentFmt.setForeground(makeQColor(COMMENT_COL));
		_rules.push_back({ QRegularExpression("#.*$"), commentFmt });

		QTextCharFormat cmdFmt;
		cmdFmt.setForeground(makeQColor(COMMAND_COL));
		cmdFmt.setFontWeight(QFont::Bold);

		QStringList commands = {
			"load",
			"start",
			"stop",
			"trajSet",
			"trajClear",
			"rotateJointTo",
			"rotateJointBy",
			"parallel",
			"set",
			"setVelocity"
		};

		for (const auto& cmd : commands) { _rules.push_back({ QRegularExpression("\\b" + cmd + "\\b"), cmdFmt }); }

		QTextCharFormat numFmt;
		numFmt.setForeground(makeQColor(NUMBER_COL));
		_rules.push_back({ QRegularExpression(R"(\b[-+]?[0-9]*\.?[0-9]+\b)"), numFmt });

		QTextCharFormat trajFmt;
		trajFmt.setForeground(makeQColor(TYPE_COL));
		QStringList trajTypes = { "TRAP", "SINE", "MSINE", "MULTISINE" };
		for (const auto& type : trajTypes) { _rules.push_back({ QRegularExpression("\\b" + type + "\\b"), trajFmt }); }

		QTextCharFormat idFmt;
		idFmt.setForeground(makeQColor(ID_COL));
		_rules.push_back({ QRegularExpression(R"(\blink[0-9]+\b)"), idFmt });
	}

	void DSLSyntaxHighlighter::highlightBlock(const QString& text) {
		int commentPos = text.indexOf('#');
		QString codeText = (commentPos >= 0) ? text.left(commentPos) : text; 

		for (const auto& rule : _rules) {
			auto it = rule.pattern.globalMatch(codeText);
			while (it.hasNext()) {
				auto match = it.next();
				setFormat(match.capturedStart(), match.capturedLength(), rule.format);
			}
		}

		if (commentPos >= 0) {
			setFormat(commentPos, text.length() - commentPos, _rules[0].format); // Comment format is the first rule
		}
	}
}