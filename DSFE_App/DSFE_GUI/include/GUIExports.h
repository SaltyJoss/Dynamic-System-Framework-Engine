// DSFE_GUI GUIExports.h
#pragma once

#ifdef _WIN32
#ifdef DSFE_GUI_EXPORTS
#define DSFE_GUI_API __declspec(dllexport)
#else
#define DSFE_GUI_API __declspec(dllimport)
#endif
#else
#define DSFE_GUI_API
#endif