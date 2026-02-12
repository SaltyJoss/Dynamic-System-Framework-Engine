# pragma once
// File:   pch.h
// GitHub: SaltyJoss

#ifndef PCH_H
#define PCH_H

// Precompiled header for EngineLib
#include "framework.h"

// standard includes
#include <string>
#include <algorithm>
#include <vector>
#include <fstream>
#include <sstream>
#include <cctype>
#include <iostream>
#include <memory>
#include <ctime>
#include <array>
#include <iomanip>
#include <mutex>
#include <filesystem>
#include <chrono>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <core/constants.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/transform.hpp>

#endif //PCH_H
