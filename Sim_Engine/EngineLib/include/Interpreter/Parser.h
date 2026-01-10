#pragma once

#include "EngineCore.h"
#include "ProgramData.h"
#include <memory>
#include <string>
#include <vector>

namespace interpreter {
	// Class representing a parsed command
	class ENGINE_API Parser {
		ProgramData parse(std::string_view code) const;
		ProgramData parseFile(const std::string& filename) const;

		static bool isBlankOrComment(std::string_view line);
	};
} // namespace interpreter