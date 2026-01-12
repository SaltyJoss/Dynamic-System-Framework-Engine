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
			//throw std::runtime_error("Cannot parse empty code string.");
		}

		_program->clear();
		
		tokenAndClassifyLine(code, _currentCmd);
		buildProgram();
	}

	// --- Handlers and Utilities ---

	std::vector<std::string> Parser::split(const std::string& s, const std::vector<std::string>& delimiters) {
		std::vector<std::string> outLines = std::vector<std::string>();

		size_t start = 0;
		size_t end = s.find_first_of(delimiters[0]);
		while (end != std::string::npos) {
			outLines.push_back(s.substr(start, end - start));
			start = end + delimiters[0].length();
			end = s.find_first_of(delimiters[0], start);
		}
		outLines.push_back(s.substr(start));

		return outLines;
	}

	void Parser::motionCommandHandler(const Command& cmd) {
		// Implementation to handle a motion command
	}

	bool Parser::isBlankOrComment(std::string_view line) {
		for (char c : line) {
			if (c == '#') return true;
			if (!std::isspace(static_cast<unsigned char>(c)))
				return false;
		}
		return true;
	}

	// --- Command and Program Builders ---

	void Parser::tokenAndClassifyLine(const std::string& code, Command& outCmd) {
		_program->setCurrentLineNumber(0);

		std::vector<std::string> delimiter = { "\n", "\n\r", "\r" };

		int start, end = -1 * delimiter[0].size();
		do {
			start = end + delimiter[0].size();							// Move past the last delimiter
			end = static_cast<int>(code.find(delimiter[0], start));	// Find the next delimiter
			std::string l = code.substr(start, end - start);		// Extract the token - token is now a line
			lines.push_back(l);								// Store the line as a token
			
		} while (end != std::string::npos); // Continue until no more delimiters are found

		for (int i = 0; i < static_cast<int>(lines.size()); ++i) {
			_program->setCurrentLineNumber(i + 1);

			std::string_view line = lines[i];

			// Trim leading whitespace
			size_t firstNonSpace = line.find_first_not_of(" \t");
			if (firstNonSpace != std::string::npos)
				line = line.substr(firstNonSpace);

			// Skip blank / comment
			if (line.empty() || line[0] == '#')
				continue;

			size_t spacePos = line.find(' ');

			Command cmd;
			cmd.rawLine = std::string(line);
			cmd.lineNumber = _program->getCurrentLineNumber();

			if (spacePos != std::string::npos) {
				cmd.cmdName = std::string(line.substr(0, spacePos));
				cmd.tokens = std::string(line.substr(spacePos + 1));
			}
			else {
				cmd.cmdName = std::string(line);
				cmd.tokens = "";
			}

			_programData.cmd.push_back(cmd);
		}
	}

	void Parser::buildProgram() {
		for (auto& cmd : _programData.cmd) {
			if (cmd.cmdName.empty())
				continue;

			buildCommand(cmd);
		}
	}

	void Parser::buildCommand(Command& cmd) {
		const auto& tokens = split(cmd.rawLine, { " " });

		if (tokens.size() < 2) {
			D_WARN("Invalid command: %s", cmd.rawLine.c_str());
			return;
		}

		cmd.identifier = tokens[1];

		std::vector<std::string> argsTokens;
		for (size_t i = 2; i < tokens.size(); ++i)
			argsTokens.push_back(tokens[i]);

		if (tokens.size() > 2) {
			argsTokens = split(tokens[2], { "," });
		}

		D_DEBUG("Command: %s Identifier: %s",
			cmd.cmdName.c_str(),
			cmd.identifier.c_str());

		auto* command =
			commands::CommandFactory::Instance()
			.create(cmd.cmdName, cmd.identifier, argsTokens);

		if (command) {
			_program->add(command);
			D_INFO("Added command: %s", cmd.cmdName.c_str());
		}
		else {
			D_FAIL("Failed to create command: %s", cmd.cmdName.c_str());
		}
	}

} // namespace interpreter