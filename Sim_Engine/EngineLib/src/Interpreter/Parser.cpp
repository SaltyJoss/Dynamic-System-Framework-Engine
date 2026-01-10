#include "pch.h"
#include "Interpreter/Parser.h"
#include "Interpreter/IStoredProgram.h"

namespace interpreter {
	ProgramData Parser::parse(std::string_view code) const {
	}

	ProgramData Parser::parseFile(const std::string& filename) const {
		std::ifstream file(filename);
		if (!file.is_open()) {
			throw std::runtime_error("Failed to open file: " + filename);
		}
		std::stringstream buffer;
		buffer << file.rdbuf();
		return parse(buffer.str());
	}

	bool Parser::isBlankOrComment(std::string_view line) {
		std::string_view trimmed = line;
		// Trim leading whitespace
		while (!trimmed.empty() && std::isspace(trimmed.front())) {
			trimmed.remove_prefix(1);
		}
		// Check if the line is empty or starts with a comment character
		return trimmed.empty() || trimmed.front() == '#';
	}

} // namespace interpreter