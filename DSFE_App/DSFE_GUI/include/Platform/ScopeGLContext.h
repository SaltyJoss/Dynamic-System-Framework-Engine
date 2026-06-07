// DSFE_GUI ScopeGLContext.h
#pragma once

#include <functional>

namespace platform {
	class ScopeGLContext {
	public:
		ScopeGLContext(std::function<void()> makeHook, std::function<void()> doneHook)
			: _makeHook(std::move(makeHook)), _doneHook(std::move(doneHook))
		{
			if (_makeHook) { _makeHook(); }
		}
		~ScopeGLContext() { if (_doneHook) { _doneHook(); } }
	private:
		std::function<void()> _makeHook;
		std::function<void()> _doneHook;
	};
}