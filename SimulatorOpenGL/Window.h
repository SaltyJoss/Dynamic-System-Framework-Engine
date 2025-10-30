#pragma once

//Basic window interface
namespace NativeWindow {
class IWindow {
public:
	virtual void* getNativeWin() = 0;
	virtual void setNativeWin(void* window) = 0;
	virtual void onResize(int w, int h) = 0;
	virtual void onClose() = 0;
};
}