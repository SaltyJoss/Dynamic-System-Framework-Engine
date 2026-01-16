#pragma once

#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include "Interpreter/Command.h"
#include "Interpreter/UIContext.h"

namespace commands {
	// Enum for preset colours
	enum class BlockColour {
		Red,
		Green,
		Blue,
		Yellow,
		Cyan,
		Magenta,
		White,
		Grey,
		DarkGrey,
		Black
	};

	// Struct for colour mapping
	struct Colour {
		BlockColour col = BlockColour::Red;
		mathlib::Vec3 rgb = { 1.0f, 0.0f, 0.0f };
	};

	// Class representing the SET command
	class ENGINE_API ColourCmd final : public Command {
	public:
		// Constructor
		ColourCmd(const std::string& id, const std::vector<std::string>& tokens);
		// Get the command name
		std::string_view getName() const { return "COLOUR"; }
		// Set the command context
		void setContext(UIContext& cntx) { _uiCntx = &cntx; }

		// Getters and Setters for Result
		CmdResult getResult() const { return _result; }
		void setResult(const CmdResult& result) { _result = result; }

		// Execute the command
		void execute() override;

		// Get colour from parameter
		Colour getColour(const std::string& parameter);

		// Set colour using different methods
		void setColour(const Colour& colour, const mathlib::Vec3& rgb);
		void setColour(const Colour& colour, BlockColour preset);
		void setColour(const Colour& colour, const std::string& hex);

	private:
		std::string _id;
		std::vector<std::string> _tokens;

		mathlib::Vec3 _defaultColRGB = { 1.0, 0.0, 0.0 }; // Default to red
		mathlib::Vec3 _currentColRGB = _defaultColRGB;
		Colour _col{ BlockColour::Red }; // Default to red

		UIContext* _uiCntx = nullptr;

		CmdResult _result = { CmdState::NotStarted, {}, "" };
	protected:
		// Mark the command as failed with a message
		void markFailed(const std::string& message) override;
		// Mark the command as completed
		void markCompleted() override;
		// Check if the command has started
		bool hasStarted() const override;
	};

	// Free function to create a ColourCmd
	std::unique_ptr<ICommand> CreateColourCmd(const std::string& id, const std::vector<std::string>& tokens);
} // namespace commands