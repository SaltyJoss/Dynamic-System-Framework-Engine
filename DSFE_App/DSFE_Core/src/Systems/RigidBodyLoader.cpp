/*
 * File: Systems/RigidBodyLoaderURDF.cpp
 * Created by: Joss Salton, 26-07-2026
 */
#include "pch.h"

#include "Systems/RigidBodyLoader.h"
#include <MathLib>

#include "EngineLib/LogMacros.h"

namespace systems {
    // Load dispatcher for all loading methods (JSON, URDF, etc.)
    RigidBodyModel RigidBodyLoader::load(const std::string& fp) {
        std::string ext;
        const auto dot = fp.find_last_of('.');
        if (dot != std::string::npos) { 
            ext = fp.substr(dot + 1);
            for (char& c : ext) { c = static_cast<char>(std::tolower((unsigned char)c)); }
        }
        if (ext == "urdf" || ext == "xml") { return loadFromURDF(fp); }
        if (ext == "json") { return loadFromJSON(fp); }
        LOG_ERROR("Unsupported file extension '%s' for rigidBody model: %s", ext.c_str(), fp.c_str());
        return RigidBodyModel();
    }
} // namespace systems