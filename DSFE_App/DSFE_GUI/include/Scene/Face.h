// DSFE_GUI Face.h
#pragma once

#include <vector>
#include "Platform/Logger.h"

using GLuint = unsigned int;

namespace scene {
    class Face {
    public:
        void addVertexIndex(GLuint index) { _vertexIndices.push_back(index); }
    private:
        std::vector<GLuint> _vertexIndices;
    };
} // namespace scene