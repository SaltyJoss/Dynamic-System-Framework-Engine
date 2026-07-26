/*
 * File: DSL/SimFwd.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include "EngineCore.h"

#include <core/Types.h>
#include <core/constants.h>
#include <cstddef>
#include <string>
#include <unordered_map>

#include "Platform/Logger.h"

// Forward declarations for the main classes used in the interpreter
namespace core { struct ISimulationCore; }
namespace systems { class RigidBodySystem; }
