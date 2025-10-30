#pragma once

//Basic window interface
namespace window {
class IWindow {
public:
	virtual void* getNativeWin() = 0;
	virtual void setNativeWin(void* window) = 0;
	virtual void onResize(int w, int h) = 0;
	virtual void onClose() = 0;

	int w;
	int h;
	std::string header;
};
}