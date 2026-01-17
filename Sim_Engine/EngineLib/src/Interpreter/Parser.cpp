#include "pch.h"
#include "Interpreter/Parser.h"
#include "Interpreter/RegisterCommand.h"
#include "Interpreter/CommandFactory.h"
#include "Interpreter/Utils.h"

#include "EngineLib/LogMacros.h"

using namespace std;
using namespace utils;

namespace interpreter {

	// --- Constructor ---
	Parser::Parser(IStoredProgram* program) : _program(program) {
		static const bool interpreterInit = [] {
			commands::RegisterAllCommands(commands::CommandFactory::Instance());
			return true;
		}();

		if (!_program) {
			D_FAIL("Parser initialized with null IStoredProgram pointer.");
			throw std::invalid_argument("Parser initialized with null IStoredProgram pointer.");
		}
	}

	// --- Parsing Methods ---

	void Parser::parse(std::string code) {
		if (code.empty()) {
			D_WARN("Cannot parse empty code string.");
			_program->stop();
			return;
		}

		_program->clear();
		_programData.cmd.clear();
		lines.clear();
		
		tokeniseAndClassifyCode(code);
		buildProgram();
	}

	// --- Handlers ---

	// Determine if a command requires an identifier
	static bool requiresIdentifier(std::string_view cmdName) {
		std::string s = toLower(cmdName);
		return	s == "spin"				||
				s == "rotateby"			||
				s == "rotateto"			||
				s == "rotatejointby"	||
				s == "rotatejointto"	||
				s == "translate"		||
				s == "set"				||
				s == "load";
	}

	// Splits a string into arguments, respecting quotes and nested braces/parentheses
	static std::vector<std::string> splitArgs(const std::string_view s) {
		std::vector<std::string> out;
		std::string current;
		current.reserve(s.size());

		// State variables (not needed previously, but trying for this new syntax!)
		char quote = 0;			// "" or ''
		char braceDepth = 0;	// {}
		char parenDepth = 0;	// ()

		// Trim whitespace from current
		auto trimInPlace = [](std::string& str) {
			auto is_ws = [](unsigned char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r'; }; // trim whitespace
			size_t a = 0;
			while (a < str.size() && is_ws(str[a])) { ++a; }
			size_t b = str.size();
			while (b > a && is_ws(str[b - 1])) { --b; }
			str = str.substr(a, b - a);
		};
		
		auto pushCurrent = [&]() {
			trimInPlace(current);
			if (!current.empty()) { out.push_back(current); }
			current.clear();
		};

		for (size_t i = 0; i < s.size(); ++i) {
			char c = s[i]; // current character

			if (quote) {
				if (quote && c == '\\' && i + 1 < s.size()) {
					current.push_back(s[i + 1]);
					++i;
					continue;
				}
				if (c == quote) { quote = 0; continue; }

				current.push_back(c);
				continue;
			}
			
			if (c == '"' || c == '\'') { quote = c; continue; }
			if (c == '{') { ++braceDepth; current.push_back(c); continue; }
			if (c == '}') { --braceDepth; current.push_back(c); continue; }
			if (c == '(') { ++parenDepth; current.push_back(c); continue; }
			if (c == ')') { --parenDepth; current.push_back(c); continue; }
			if (c == ',' && braceDepth == 0 && parenDepth == 0) { pushCurrent(); continue; }

			current.push_back(c);
		}
		pushCurrent();
		return out;
	}

	// --- Line Analyzers ---

	// Check if a line is blank or a comment
	bool Parser::isBlankOrComment(std::string_view line) {
		line = trim(line);
		return line.empty() || line[0] == '#';
	}
	// Check if a line has an inline comment
	bool Parser::hasCommentInline(const std::string_view s) {
		for (char c : s) {
			if (c == '#') { return true; }
		}
		return false;
	}

	// --- Command and Program Builders ---

	// Tokenise and classify code into commands
	void Parser::tokeniseAndClassifyCode(const std::string& code) {
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
				size_t commentPos = line.find('//');
				line = line.substr(0, commentPos);
			}
			// NEW SYNTAX PARSING
			{
				Command cmd;
				cmd.rawLine = std::string(line);
				cmd.lineNumber = _program->getCurrentLineNumber();

				// Find positions of '(' and ')'
				size_t open = line.find('(');
				size_t close = line.rfind(')');

				// Validate positions
				if (open == std::string_view::npos || close == std::string_view::npos || close < open) {
					D_WARN("Invalid DSL syntax (line %d): %s", cmd.lineNumber, cmd.rawLine.c_str());
					_program->stop();
					return;
				}

				// Extract command name
				cmd.cmdName = std::string(toLower(trim(line.substr(0, open))));
				if (cmd.cmdName.empty()) {
					D_WARN("Missing command name (line %d): %s", cmd.lineNumber, cmd.rawLine.c_str());
					_program->stop();
					return;
				}

				// Extract inside of parentheses
				std::string_view inside = line.substr(open + 1, close - open - 1);
				auto parts = splitArgs(inside);
				
				// Process parts based on whether an identifier is required
				if (requiresIdentifier(cmd.cmdName)) {
					if (parts.empty()) {
						D_WARN("Command '%s' requires an identifier (line %d): %s", cmd.cmdName.c_str(), cmd.lineNumber, cmd.rawLine.c_str());
						_program->stop();
						return;
					}
					// first part is identifier
					cmd.identifier = std::string(toLower(parts[0]));	// first part is identifier
					cmd.tokens.assign(parts.begin() + 1, parts.end());	// remaining parts are tokens
				} else { // no identifier required
					cmd.identifier.clear();
					cmd.tokens = std::move(parts);
				}
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
			_program->stop();
			return;
		}

		// Prepare identifier and tokens
		if (requiresIdentifier(cmd.cmdName) && cmd.identifier.empty()) {
			D_FAIL("Command '%s' requires an identifier (line %d)", cmd.cmdName.c_str(), cmd.lineNumber);
			_program->stop();
			return;
		}

		D_DEBUG("Command: %s Identifier: %s Args: %d",
			cmd.cmdName.c_str(),
			cmd.identifier.c_str(),
			(int)cmd.tokens.size());

		for (const auto& t : cmd.tokens) {
			D_TRACE("Arg: %s", t.c_str());
		}

		auto* command = commands::CommandFactory::Instance().create(cmd.cmdName, cmd.identifier, cmd.tokens);

		if (command) {
			_program->add(command);
			D_INFO("Added command: %s()", cmd.cmdName.c_str());
		}
		else {
			D_FAIL("Failed to create command: %s()", cmd.cmdName.c_str());
			_program->stop();
			return;
		}
	}

} // namespace interpreter

// OLD SYNTAX: "ROTATE <objID>/<axis> <omega>,<startDeg>,<endDeg>"
// NEW SYNTAX: "rotate(<identifier>, <omega>,<startDeg>,<endDeg>"
// identifier could be <objID> or <{x,y,z}> or <"name">, for objects, axes, or robots respectively

