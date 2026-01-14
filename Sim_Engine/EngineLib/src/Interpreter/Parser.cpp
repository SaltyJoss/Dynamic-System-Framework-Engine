#include "pch.h"
#include "Interpreter/Parser.h"
#include "Interpreter/RegisterCommand.h"
#include "Interpreter/CommandFactory.h"

#include "EngineLib/LogMacros.h"

using namespace std;

namespace interpreter {

	// --- Constructor ---
	Parser::Parser(IStoredProgram* program) : _currentCmd(*(new Command())), _programData(*(new ProgramData())), lines(*(new std::vector<std::string>())) {
		static const bool interpreterInit = [] {
			commands::RegisterAllCommands(commands::CommandFactory::Instance());
			return true;
		}();

		_program = program;
		if (_program == nullptr) {
			D_FAIL("Parser initialized with null IStoredProgram pointer.");
			throw std::invalid_argument("Parser initialized with null IStoredProgram pointer.");
		}
	}

	// --- Parsing Methods ---

	void Parser::parse(std::string code) {
		if (code.empty()) {
			D_WARN("Cannot parse empty code string.");
			return;
		}

		_program->clear();
		_programData.cmd.clear();
		lines.clear();
		
		tokeniseAndClassifyLine(code, _currentCmd);
		buildProgram();
	}

	// --- Handlers and Utilities ---

	std::vector<std::string> Parser::split(const std::string_view s, const std::string_view delims) {
		std::vector<std::string> out;

		size_t start = 0;

		auto push_token = [&](size_t a, size_t b) {
			if (b > a) { out.emplace_back(s.substr(a, b - a)); }
		};

		for (size_t i = 0; i < s.size(); ++i) {
			if (delims.find(s[i])) {
				push_token(start, i);
				start = i + 1;
			}
		}
		push_token(start, s.size());

		return out;
	}

	std::string_view Parser::trim(std::string_view str) {
		size_t a = str.find_first_not_of(" \t\r");
		if (a == std::string_view::npos) { return ""; } // All whitespace
		size_t b = str.find_last_not_of(" \t\r");
		return str.substr(a, b - a + 1);
	}

	bool Parser::isBlankOrComment(std::string_view line) {
		line = trim(line);
		return line.empty() || line[0] == '#';
	}

	bool Parser::hasWhitespace(const std::string_view s) {
		for (char c : s) {
			if (c == ' ' || c == '\t') { return true; }
		}
		return false;
	}

	bool Parser::hasCommentInline(const std::string_view s) {
		for (char c : s) {
			if (c == '#') { return true; }
		}
		return false;
	}


	void Parser::motionCommandHandler(const Command& cmd) {
		// Implementation to handle a motion command
	}

	// --- Command and Program Builders ---

	void Parser::tokeniseAndClassifyLine(const std::string& code) {
		_program->setCurrentLineNumber(0);
		
		// Splits code into lines - right now I need to handle the different line endings
		size_t start = 0;
		while (start <= code.size()) {
			size_t end = code.find('\n', start);
			std::string_view line = (end == std::string::npos) 
				? std::string_view(code).substr(start) 
				: std::string_view(code).substr(start, end - start);

			if (!line.empty() && line.back() == '\r') { line = line.substr(0, line.size() - 1); }
			lines.emplace_back(line);

			if (end == std::string::npos) { break; }
			start = end + 1;
		}


		for (int i = 0; i < static_cast<int>(lines.size()); ++i) {
			_program->setCurrentLineNumber(i + 1);
			std::string_view line = lines[i];

			if (isBlankOrComment(line)) { continue; }
			if (hasCommentInline(line)) {
				size_t commentPos = line.find('#');
				line = line.substr(0, commentPos);
			}

			{
				Command cmd;
				cmd.rawLine = std::string(line);

				// Tokenize line on '(', ')', ',', and ' ' characters for new syntax!
				auto parts = split(line, "(), ");
				if (parts.size() < 2 || parts[0].empty() || parts[1].empty()) {
					D_WARN("Invalid DSL syntax (line %d): %s",
						_program->getCurrentLineNumber(),
						cmd.rawLine.c_str());
					goto next_line;
				}
				cmd.cmdName = parts[0];
				cmd.identifier = parts[1];

				for (size_t k = 2; k < parts.size(); ++k) {
					if (!parts[k].empty()) { cmd.tokens.emplace_back(parts[k]); }
				}

				cmd.lineNumber = _program->getCurrentLineNumber();
				_programData.cmd.push_back(std::move(cmd)); // Store the command
			}

		next_line:
			continue;
		}
	}

	void Parser::buildProgram() {
		for (auto& cmd : _programData.cmd) {
			if (cmd.cmdName.empty())
				continue;

			D_DEBUG("SCRIPT: %s \n\t| target=%s \n\t| args=%d", 
				cmd.cmdName.c_str(),
				cmd.identifier.c_str(),
				cmd.tokens.size()
			);

			buildCommand(cmd);
		}
	}

	void Parser::buildCommand(Command& cmd) {
		if (cmd.cmdName.empty() || cmd.identifier.empty()) {
			D_WARN("Invalid command fields at line %d", cmd.lineNumber);
			return;
		}

		D_DEBUG("Command: %s Identifier: %s Args: %d",
			cmd.cmdName.c_str(),
			cmd.identifier.c_str(),
			(int)cmd.tokens.size());

		auto* command =
			commands::CommandFactory::Instance().create(cmd.cmdName, cmd.identifier, cmd.tokens);

		if (command) {
			_program->add(command);
			D_INFO("Added command: %s()", cmd.cmdName.c_str());
		}
		else {
			D_FAIL("Failed to create command: %s()", cmd.cmdName.c_str());
		}
	}

} // namespace interpreter

// OLD SYNTAX: "ROTATE <objID>/<axis> <omega>,<startDeg>,<endDeg>"
// NEW SYNTAX: "rotate(<identifier>, <omega>,<startDeg>,<endDeg>"
// identifier could be <objID> or <{x,y,z}> or <"name">, for objects, axes, or robots respectively

