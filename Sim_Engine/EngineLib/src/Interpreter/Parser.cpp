#include "pch.h"
#include "Interpreter/Parser.h"
#include "Interpreter/IStoredProgram.h"

using namespace std;

namespace interpreter {
	ProgramData Parser::parse(std::string code) const {
		ProgramData program;
		std::istringstream ss(code);
		std::string line;

		size_t lineNumber = 0;

		while (std::getline(ss, line)) {
			if (isBlankOrComment(line)) {
				continue; // Skip blank lines and comments
			}

			std::istringstream ls(line);
			Instruction inst;

			ls >> inst.name;
			std::string arg;

			while (ls >> arg) {
				inst.args.push_back(arg);
			}

			// source location (minimal but valid)
			inst.loc.line = lineNumber;
			inst.loc.column = 1;

			program.instructions.push_back(inst);
			lineNumber++;
		}
		return program;
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
		for (char c : line) {
			if (c == '#') return true;
			if (!std::isspace(static_cast<unsigned char>(c)))
				return false;
		}
		return true;
	}

} // namespace interpreter