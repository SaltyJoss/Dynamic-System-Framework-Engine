// DSFE_GUI ScopeGLContext.h
#pragma once

#include <functional>

namespace platform {
	class ScopeGLContext {
	public:
		ScopeGLContext(std::function<void()> make, std::function<void()> done) : _done(std::move(done)) { if (make) { make(); } }
		~ScopeGLContext() { if (_done) { _done(); } }
	private:
		std::function<void()> _done;
	};
}