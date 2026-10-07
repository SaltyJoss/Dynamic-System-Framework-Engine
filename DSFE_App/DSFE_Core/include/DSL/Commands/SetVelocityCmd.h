/*
 * File: DSL/Commands/SetVelocityCmd.h
 * Created by: Joss Salton, 26-07-2026 
 */
#include "EngineCore.h"

#include "DSL/SimFwd.h"
#include "DSL/Command.h"
#include "DSL/CommandContext.h"

namespace commands {
    class SetVelocityCmd final : public Command {
        public:
            SetVelocityCmd(std::string link, double wx, double wy, double wz, double vx, double vy, double vz);
            ~SetVelocityCmd() override = default;

            std::string_view getName() const { return "setVelocity"; }
            void setContext(CommandContext& cntx) override { _cntx = &cntx; }
            CmdResult getResult() const { return _result; }
            void setResult(const CmdResult& result) { _result = result; }
            CmdResult currentResult() const override { return getResult(); }

        private:
            CmdResult update(CommandContext& cntx, double dt) override;
            void execute() override;

            std::string _link;
            double _wx, _wy, _wz; // angular velocity components
            double _vx, _vy, _vz; // linear velocity components
            bool _done = false;
            bool _started = false;
            CmdResult _result{ CmdState::NotStarted, {}, "" };

        protected:
            void markFailed(const std::string& message) override;
            void markCompleted() override;
            bool hasStarted() const override;
    };
    std::unique_ptr<ICommand> CreateSetVelocityCmd(const std::string& id, const std::vector<std::string>& args);
}