#pragma once

#include "EngineCore.h"
#include "Interpreter/SimFwd.h"
#include "Interpreter/Command.h"
#include "Interpreter/CommandContextMotion.h"

#include "Platform/Logger.h"

namespace commands {

    class TrajSetCmd final : public Command {
    public:
        TrajSetCmd(std::string link, std::string type, std::vector<double> params);
        ~TrajSetCmd() override = default;

        std::string_view getName() const { return "trajSet"; }
        void setContext(CommandContextMotion& cntx) override { _cntxMtn = &cntx; }
        program_data::CmdResult getResult() const { return _result; }
        void setResult(const program_data::CmdResult& result) { _result = result; }
        program_data::CmdResult currentResult() const override { return getResult(); }

    private:
        program_data::CmdResult update(CommandContextMotion& cntx, double dt) override;
        void execute() override;

        CommandContextMotion* _cntxMtn = nullptr;

        std::string _link;
        std::string _type;
        std::vector<double> _params;

        bool _done = false;
        bool _started = false;

        program_data::CmdResult _result{ CmdState::NotStarted, {}, "" };

        static std::string upperCopy(std::string s);

    protected:
        // Mark the command as failed with a message
        void markFailed(const std::string& message) override;
        // Mark the command as completed
        void markCompleted() override;
        // Check if the command has started
        bool hasStarted() const override;
    };

    std::unique_ptr<ICommand> CreateTrajSetCmd(const std::string& id, const std::vector<std::string>& args);

} // namespace commandsnds