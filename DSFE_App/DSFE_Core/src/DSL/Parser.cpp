/*
 * File: DSL/Parser.cpp
 * Created by: Joss Salton, 26-07-2026
 */
#include "pch.h"

#include "DSL/Parser.h"
#include "DSL/RegisterCommand.h"
#include "DSL/CommandFactory.h"
#include "DSL/Commands/ParallelGroupCmd.h"
#include "DSL/Utils.h"

#include "EngineLib/LogMacros.h"

using namespace std;
using namespace utils;

namespace dsl {
	// --- Handlers ---

	// Determine if a command requires an identifier
	bool Parser::requiresIdentifier(std::string_view cmdName) {
		std::string s = toLower(cmdName);
		return	s == "spin"			 ||
				s == "rotateby"		 ||
				s == "rotateto"		 ||
				s == "rotatejointby" ||
				s == "rotatejointto" ||
				s == "trajset"		 ||
				s == "translate"	 ||
				s == "set"			 ||
				s == "select" 		 ||
				s == "setomega"		 ||
				s == "setvelocity"   ||
				s == "load";
	}

	// Check if a string is a valid identifier
	bool Parser::matchIdentifier(const std::string& s) {
		if (s.empty()) return false;
		if (!isalpha(s[0]) && s[0] != '_') { return false; }
		for (char c : s) { if (!isalnum(c) && c != '_') return false; }
		return true;
	}

	// Splits a string into arguments, respecting quotes and nested braces/parentheses
	static std::vector<std::string> splitArgs(const std::string_view s) {
		std::vector<std::string> out;
		std::string current;
		current.reserve(s.size());

		// State variables (not needed previously, but trying for this new syntax!)
		char quote = 0;			// "" or ''
		int braceDepth = 0;	// {}
		int parenDepth = 0;	// ()

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
			// Braces
			if (c == '{') { ++braceDepth; current.push_back(c); continue; }
			if (c == '}') { 
				if (braceDepth > 0) { --braceDepth; } 
				else { D_WARN("Unmatched closing brace '}' in argument list"); }
				current.push_back(c); continue; 
			}
			// Parentheses
			if (c == '(') { ++parenDepth; current.push_back(c); continue; }
			if (c == ')') { 
				if (parenDepth > 0) { --parenDepth; } 
				else { D_WARN("Unmatched closing parenthesis ')' in argument list"); }
				current.push_back(c); continue; 
			}
			// Comma (only if not nested)
			if (c == ',' && braceDepth == 0 && parenDepth == 0) { pushCurrent(); continue; }

			current.push_back(c);
		}
		pushCurrent();
		return out;
	}

	// --- Line Analyzers ---

	// Check if a string starts with a specific word (case-insensitive)
	static bool startsWithWord(std::string_view s, std::string_view word) {
		s = utils::trim(s);
		if (s.size() < word.size()) return false;
		if (utils::toLower(s.substr(0, word.size())) != word) return false;
		// next char must be whitespace, '(' or '{' or end
		if (s.size() == word.size()) return true;
		char c = s[word.size()];
		return c == ' ' || c == '\t' || c == '(' || c == '{';
	}

	// Parse timeout value from a parallel(...) line
	static double parseParallelTimeoutFromLine(std::string_view line, double def = 0.0) {
		line = utils::trim(line);
		// expecting "parallel(...)" or "parallel"
		auto open = line.find('(');
		if (open == std::string_view::npos) return def;
		auto close = line.find(')', open);
		if (close == std::string_view::npos) return def;

		auto inside = utils::trim(line.substr(open + 1, close - open - 1));
		if (inside.empty()) return def;

		// stod needs std::string
		try { return std::stod(std::string(inside)); }
		catch (...) { return def; }
	}

	// Check if a line is blank or a comment
	static bool isBlankOrComment(std::string_view line) {
		line = trim(line);
		return line.empty() || line[0] == '#';
	}// Check if a line has an inline comment
	static bool hasCommentInline(const std::string_view s) {
		for (char c : s) { if (c == '#') { return true; } }
		return false;
	}

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

		for (int i = 0; i < (int)lines.size(); ++i) {
			_program->setCurrentLineNumber(i + 1);
			std::string_view line = lines[i];

			if (isBlankOrComment(line)) continue;

			if (hasCommentInline(line)) {
				size_t commentPos = line.find('#');
				line = line.substr(0, commentPos);
			}
			line = trim(line);
			if (line.empty()) continue;

			// ---- PARALLEL BLOCK ----
			if (startsWithWord(line, "parallel")) {
				// parse timeout from "parallel(2.0)"
				double timeoutSec = parseParallelTimeoutFromLine(line, 0.0);

				// ensure '{' exists on this line or the next nonblank line
				bool hasLBrace = (line.find('{') != std::string_view::npos);

				// If '{' not on same line, scan forward to find it
				while (!hasLBrace) {
					int j = i + 1;
					while (j < (int)lines.size() && (isBlankOrComment(lines[j]) || trim(lines[j]).empty())) j++;
					if (j >= (int)lines.size()) { D_FAIL("parallel missing '{'"); _program->stop(); return; }
					i = j; // advance outer index to brace line
					_program->setCurrentLineNumber(i + 1);
					auto braceLine = trim(lines[i]);
					if (hasCommentInline(braceLine)) braceLine = trim(braceLine.substr(0, braceLine.find('#')));
					hasLBrace = (braceLine.find('{') != std::string_view::npos);
					if (!hasLBrace) { D_FAIL("parallel missing '{'"); _program->stop(); return; }
				}

				// inner commands
				std::vector<dsl::Command> innerCmds;
				int braceDepth = 1;

				// consume subsequent lines until matching '}'
				while (++i < (int)lines.size()) {
					_program->setCurrentLineNumber(i + 1);
					std::string_view innerLine = lines[i];

					if (isBlankOrComment(innerLine)) continue;
					if (hasCommentInline(innerLine)) {
						size_t commentPos = innerLine.find('#');
						innerLine = innerLine.substr(0, commentPos);
					}
					innerLine = trim(innerLine);
					if (innerLine.empty()) continue;

					// update brace depth
					std::string_view t = innerLine;
					if (t == "{") { braceDepth++; continue; }
					if (t == "}") { braceDepth--; if (braceDepth == 0) break; continue; }

					// parse inner command line
					dsl::Command cmd;
					cmd.rawLine = std::string(innerLine);
					cmd.lineNumber = _program->getCurrentLineNumber();

					size_t open = innerLine.find('(');
					size_t close = innerLine.rfind(')');

					if (open == std::string_view::npos || close == std::string_view::npos || close < open) {
						LOG_ERROR("Invalid DSL syntax in parallel block (line %d): %s", cmd.lineNumber, cmd.rawLine.c_str());
						_program->stop(); return;
					}

					// extract command name
					cmd.cmdName = std::string(toLower(trim(innerLine.substr(0, open))));
					std::string_view inside = innerLine.substr(open + 1, close - open - 1);
					auto parts = splitArgs(inside);

					if (requiresIdentifier(cmd.cmdName)) {
						if (parts.empty()) {
							LOG_ERROR("Command '%s' requires identifier inside parallel (line %d)", cmd.cmdName.c_str(), cmd.lineNumber);
							_program->stop(); return;
						}
						cmd.identifier = std::string(toLower(parts[0]));
						cmd.tokens.assign(parts.begin() + 1, parts.end());
					}
					else {
						cmd.identifier.clear();
						cmd.tokens = std::move(parts);
					}

					cmd.isParallelBlock = false;		 // not a parallel block
					innerCmds.push_back(std::move(cmd)); // store inner command
				}

				if (braceDepth != 0) {
					LOG_ERROR("parallel block missing closing '}'");
					_program->stop(); 
					return;
				}

				dsl::Command par;
				par.cmdName = "parallel";
				par.rawLine = std::string(line);
				par.lineNumber = _program->getCurrentLineNumber();
				par.isParallelBlock = true;
				par.timeoutSec = timeoutSec;
				par.inner = std::move(innerCmds);

				_programData.cmd.push_back(std::move(par));
				continue;
			}
			{
				Command cmd;
				cmd.rawLine = std::string(line);
				cmd.lineNumber = _program->getCurrentLineNumber();

				// Find positions of '(' and ')'
				size_t open = line.find('(');
				size_t close = line.rfind(')');

				// Validate positions
				if (open == std::string_view::npos || close == std::string_view::npos || close < open) {
					LOG_WARN("Invalid DSL syntax (line %d): %s", cmd.lineNumber, cmd.rawLine.c_str());
					_program->stop();
					return;
				}

				// Extract command name
				cmd.cmdName = std::string(toLower(trim(line.substr(0, open))));
				if (cmd.cmdName.empty()) {
					LOG_WARN("Missing command name (line %d): %s", cmd.lineNumber, cmd.rawLine.c_str());
					_program->stop();
					return;
				}

				// Extract inside of parentheses
				std::string_view inside = line.substr(open + 1, close - open - 1);
				auto parts = splitArgs(inside);
				
				// Process parts based on whether an identifier is required
				if (requiresIdentifier(cmd.cmdName)) {
					if (parts.empty()) {
						LOG_WARN("Command '%s' requires an identifier (line %d): %s", cmd.cmdName.c_str(), cmd.lineNumber, cmd.rawLine.c_str());
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

				//LOG_INFO("PARSE cmdName='%s' raw='%s'", cmd.cmdName.c_str(), cmd.rawLine.c_str());

				_programData.cmd.push_back(std::move(cmd)); // Store the command
			}
		}
	}

	// --- Command and Program Builders ---

	void Parser::buildProgram() {
		for (auto& cmd : _programData.cmd) {
			if (cmd.cmdName.empty()) { continue; }
			if (cmd.isParallelBlock) {
				std::vector<std::unique_ptr<commands::ICommand>> innerCmds;
				innerCmds.reserve(cmd.inner.size());

				// Build inner commands
				for (auto& innerCmdData : cmd.inner) {
					commands::ICommand* raw = commands::CommandFactory::Instance().create(innerCmdData.cmdName, innerCmdData.identifier, innerCmdData.tokens);
					if (!raw) { _program->stop(); return; }
					innerCmds.emplace_back(raw);
				}

				// Create ParallelGroupCmd
				auto group = std::make_unique<commands::ParallelGroupCmd>(commands::ParallelGroupCmd::Policy::All, std::move(innerCmds), cmd.timeoutSec );
				_program->add(std::move(group));

				//LOG_INFO("CREATE CMD: %s | target=%s | args=%d | inner cmds=%zu", cmd.cmdName.c_str(), "", 0, cmd.inner.size());
				continue;
			}
			buildCommand(cmd);
		}
	}

	void Parser::buildCommand(Command& cmd) {
		if (cmd.cmdName.empty()) {
			LOG_WARN("Invalid command fields at line %d", cmd.lineNumber);
			_program->stop();
			return;
		}

		// Prepare identifier and tokens
		if (requiresIdentifier(cmd.cmdName) && cmd.identifier.empty()) {
			LOG_ERROR("Command '%s' requires an identifier (line %d)", cmd.cmdName.c_str(), cmd.lineNumber);
			_program->stop();
			return;
		}

		// Check if command is registered
		if (!commands::CommandFactory::Instance().hasCommand(cmd.cmdName)) {
			LOG_ERROR("Unknown command: %s (line %d)", cmd.cmdName.c_str(), cmd.lineNumber);
			_program->stop();
			return;
		}
		
		// Create command instance
		auto* command = commands::CommandFactory::Instance().create(cmd.cmdName, cmd.identifier, cmd.tokens);

		//LOG_INFO("CREATE CMD: %s | target=%s | args=%d | inner cmds=%zu", cmd.cmdName.c_str(), "", 0, cmd.inner.size());

		if (command) {
			D_DEBUG("SCRIPT: %s | target=%s | args=%d", cmd.cmdName.c_str(), cmd.identifier.c_str(), cmd.tokens.size());
			_program->add(command);
			//D_INFO("Added command: %s()", cmd.cmdName.c_str());
		}
		else {
			LOG_ERROR("Failed to create command: %s()", cmd.cmdName.c_str());
			_program->stop();
			return;
		}
	}

	// --- Constructor ---
	Parser::Parser(IStoredProgram* program) : _program(program) {
		static const bool interpreterInit = [] {
			commands::RegisterAllCommands(commands::CommandFactory::Instance());
			return true;
		}();

		if (!_program) {
			LOG_ERROR("Parser initialised with null IStoredProgram pointer.");
			throw std::invalid_argument("Parser initialized with null IStoredProgram pointer.");
		}
	}

	// --- Parsing Methods ---

	void Parser::parse(std::string code) {
		if (code.empty()) {
			LOG_WARN("Cannot parse empty code string.");
			_program->stop();
			return;
		}

		_program->clear();
		_programData.cmd.clear();
		lines.clear();

		tokeniseAndClassifyCode(code);
		buildProgram();
	}

} // namespace dsl

