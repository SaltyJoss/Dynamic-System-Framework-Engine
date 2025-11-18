#pragma once

#ifdef MATHLIB_BUILD
#define MATHLIB_API __declspec(dllexport)
#else
#define MATHLIB_API __declspec(dllimport)
#endif