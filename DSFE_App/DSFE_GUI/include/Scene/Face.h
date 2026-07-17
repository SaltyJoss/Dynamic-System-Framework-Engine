// DSFE_GUI Face.h
#pragma once

#include <vector>
#include "Platform/Logger.h"

namespace scene {
    class Face {
    public:
        void addVertexIndex(uint32_t index) { _vertexIndices.push_back(index); }
    private:
        std::vector<uint32_t> _vertexIndices; // Indices of vertices that make up this face
    };
} // namespace scene