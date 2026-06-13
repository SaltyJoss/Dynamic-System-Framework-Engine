// pch.h: This is a precompiled header file.
// Files listed below are compiled only once, improving build performance for future builds.
// This also affects IntelliSense performance, including code completion and many code browsing features.
// However, files listed here are ALL re-compiled if any one of them is updated between builds.
// Do not add files here that you will be updating frequently as this negates the performance advantage.

#ifndef PCH_H
#define PCH_H

#include "framework.h"

// Standard Libraries
#include <string>
#include <algorithm>
#include <functional>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <memory>
#include <ctime>
#include <array>

#include <core/MathLib.h> // All of PhysLib depends on MathLib, necessary here to avoid circular dependencies

#endif //PCH_H
