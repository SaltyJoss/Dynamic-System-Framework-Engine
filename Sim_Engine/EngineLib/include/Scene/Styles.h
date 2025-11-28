#pragma once

// =============================================
//            File: Styles.h
// =============================================
// GUI Styles for dark and light modes.
//
// Summary:
// =============================================
// 
// public:
// --------------------------------------------
// void DarkMode()
//      -> Applies the dark mode style to the GUI.
// void LightMode()
//      -> Applies the light mode style to the GUI.
// --------------------------------------------
//
// ============================================
//              GitHub: SaltyJoss
// ============================================

#include "EngineCore.h"

namespace gui {
	class Styles {
	public:
		/// DARK MODE STYLE (VS 2022 esc)
		void DarkMode();

		/// LIGHT MODE STYLE (VS 2022 esc)
		void LightMode();
	};
}
