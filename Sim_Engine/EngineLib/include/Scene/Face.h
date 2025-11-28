#pragma once

// =============================================
//            File: Face.h
// =============================================
// Class representing a face in a 3D mesh, defined by vertex indices.
//
// Summary:
// =============================================
// 
// public:
// --------------------------------------------
// void addVertexIndex(GLuint index)
//      -> Adds a vertex index to the face.
// --------------------------------------------
// 
// private:
// --------------------------------------------
// std::vector<GLuint> _vertexIndices
//      -> Vector storing the vertex indices that make up the face.
// --------------------------------------------
//
// ============================================
//			  GitHub: SaltyJoss
// ============================================

#include "EngineCore.h"

#include <vector>
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

using GLuint = unsigned int;

namespace scene {
    class ENGINE_API Face {
    public:
        void addVertexIndex(GLuint index) { _vertexIndices.push_back(index); }

    private:
        std::vector<GLuint> _vertexIndices;
    };
}