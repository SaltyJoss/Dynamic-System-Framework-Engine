#pragma once
// File:   Face.h
// GitHub: SaltyJoss
#include "EngineCore.h"

#include <vector>
#include "Platform/Logger.h"

extern DSFE_API Debug gLog;

using GLuint = unsigned int;

namespace scene {
    class DSFE_API Face {
    public:
        void addVertexIndex(GLuint index) { _vertexIndices.push_back(index); }
    private:
        std::vector<GLuint> _vertexIndices;
    };
} // namespace scene