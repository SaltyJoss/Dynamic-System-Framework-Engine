/*
 NOT CURRENT USED IN THE PROJECT
*/

#ifndef TITLE_BAR_PANEL_H
#define TITLE_BAR_PANEL_H

#include "CoreIncludes.h"
#include "WindowManager.h"

class TitleBarPanel {
public:
	TitleBarPanel(WindowManager* windowManger);
	TitleBarPanel();
	void Render(float height);

private:
	WindowManager* windowManager;

	void InitIcons();
	void DrawMinimiseButton(float size);
	void DrawMaximiseButton(float size);
	void DrawCloseButton(float size);

	GLuint iconMinimiseTex = 0;
	GLuint iconMaximiseTex = 0;
	GLuint iconCloseTex = 0;
};

#endif