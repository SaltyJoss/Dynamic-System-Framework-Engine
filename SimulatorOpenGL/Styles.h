#ifndef STYLES_H
#define STYLES_H

#include "CoreIncludes.h"


class StyleModes {
public:
	ImVec4* colors = style.Colors;
	ImGuiStyle& style = ImGui::GetStyle();

	/// DARK MODE STYLE (VS 2022 esc)
	void DarkMode();

	/// LIGHT MODE STYLE (VS 2022 esc)
	void LightMode();
};

#endif