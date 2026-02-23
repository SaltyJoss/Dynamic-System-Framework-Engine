#pragma once
// File:    TrajClearCmd.h
// GitHub:  SaltyJoss
#include "EngineCore.h"
#include "Interpreter/SimFwd.h"
#include "Interpreter/Command.h"
#include "Interpreter/CommandContextMotion.h"

namespace commands {

    class TrajClearCmd final : public Command {
    public:
        explicit TrajClearCmd() = default;
        ~TrajClearCmd() override = default;

        std::string_view getName() const { return "trajClear"; }
        void setContext(CommandContextMotion& cntx) override { _cntxMtn = &cntx; }
        program_data::CmdResult getResult() const { return _result; }
        void setResult(const program_data::CmdResult& result) { _result = result; }
        program_data::CmdResult currentResult() const override { return getResult(); }

    private:
        program_data::CmdResult update(CommandContextMotion& cntx, double dt) override;
        void execute() override;

		core::ISimulationCore* _core = nullptr;

        bool _done = false;
        bool _started = false;

        program_data::CmdResult _result{ CmdState::NotStarted, {}, "" };

    protected:
        // Mark the command as failed with a message
        void markFailed(const std::string& message) override;
        // Mark the command as completed
        void markCompleted() override;
        // Check if the command has started
        bool hasStarted() const override;
    };

    std::unique_ptr<ICommand> CreateTrajClearCmd(const std::string& id, const std::vector<std::string>& args);

} // namespace commands