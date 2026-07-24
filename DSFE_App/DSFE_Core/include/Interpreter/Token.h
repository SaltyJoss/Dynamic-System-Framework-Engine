// DSFE_Core Token.h
#pragma once

#include "EngineCore.h"
#include <string>

namespace interpreter {
	enum class TokenType {
		Unknown,
		Comment,
		Identifier,
		Number,
		LParen,
		RParen,
		LBrace,
		RBrace,
		Comma,
		EndOfLine,
		EndOfFile,
		String
	};

	// Struct representing a token in the parser
	struct DSFE_API Token {
		TokenType type = TokenType::Unknown;
		std::string value;
		double numberValue = 0.0;
		int lineNumber = 0;
		int columnNumber = 0;
	};
} // namespace interpreter