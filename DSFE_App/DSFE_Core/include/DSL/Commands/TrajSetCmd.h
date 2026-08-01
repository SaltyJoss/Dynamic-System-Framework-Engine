/*
 * File: DSL/Commands/TrajSetCmd.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include "EngineCore.h"

#include "DSL/SimFwd.h"
#include "DSL/Command.h"
#include "DSL/CommandContext.h"

namespace commands {
    class TrajSetCmd final : public Command {
    public:
        TrajSetCmd(std::string link, std::string type, std::vector<double> params);
        ~TrajSetCmd() override = default;

        std::string_view getName() const { return "trajSet"; }
        void setContext(CommandContext& cntx) override { _cntx = &cntx; }
        CmdResult getResult() const { return _result; }
        void setResult(const CmdResult& result) { _result = result; }
        CmdResult currentResult() const override { return getResult(); }

    private:
        CmdResult update(CommandContext& cntx, double dt) override;
        void execute() override;

        std::string _link;
        std::string _type;
        std::vector<double> _params;
        bool _done = false;
        bool _started = false;
        CmdResult _result{ CmdState::NotStarted, {}, "" };

        static std::string upperCopy(std::string s);

    protected:
        void markFailed(const std::string& message) override;
        void markCompleted() override;
        bool hasStarted() const override;
    };

    std::unique_ptr<ICommand> CreateTrajSetCmd(const std::string& id, const std::vector<std::string>& args);

} // namespace commands