#include "pch.h"
// File:   importObj.cpp
// GitHub: SaltyJoss
#include "Assets/importObj.h"
#include "Assets/VertexHolder.h"
#include "Platform/str_utils.h"

#include "EngineLib/LogMacros.h"

namespace mesh_import {
	// Simple OBJ file importer that reads vertex positions, normals, UVs, and faces to populate a Mesh object
    bool ObjMeshImporter::fromFile(const std::string& filepath, scene::Mesh* pMesh) {
        if (!pMesh) {
            LOG_ERROR("OBJ importer: pMesh was NULL");
            return false;
        }

        std::ifstream in(filepath);
        if (!in.is_open()) {
            LOG_ERROR("Failed to open OBJ file: %s", filepath.c_str());
            return false;
        }

        LOG_INFO("Importing OBJ file: %s", filepath.c_str());

        // Temporary OBJ attribute arrays
        std::vector<glm::vec3> temp_positions;
        std::vector<glm::vec3> temp_normals;
        std::vector<glm::vec2> temp_uvs;

        pMesh->clean();  // wipe old mesh data

		// Read the OBJ file line by line
        std::string line;
        while (std::getline(in, line)) {
            std::istringstream ss(line);
            std::string header;
            ss >> header;

			// Vertex position
            if (header == "v") {
                glm::vec3 pos;
                ss >> pos.x >> pos.y >> pos.z;
                temp_positions.push_back(pos);
            }

			// Texture coordinate
            else if (header == "vt") {
                glm::vec2 uv;
                ss >> uv.x >> uv.y;
                temp_uvs.push_back(uv);
            }

			// Vertex normal
            else if (header == "vn") {
                glm::vec3 n;
                ss >> n.x >> n.y >> n.z;
                temp_normals.push_back(n);
            }

			// Face
            else if (header == "f") {
                std::string f1, f2, f3;
                ss >> f1 >> f2 >> f3;
                std::vector<std::string> faces = { f1, f2, f3 };

				// Each face token can be in the format: v, v/vt, v//vn, or v/vt/vn
                for (auto& f : faces) {
                    auto toks = utils::tokenize(f, '/');

                    int vIdx = toks.size() > 0 ? (int)toks[0] - 1 : -1;
                    int vtIdx = toks.size() > 1 ? (int)toks[1] - 1 : -1;
                    int vnIdx = toks.size() > 2 ? (int)toks[2] - 1 : -1;

                    if (vIdx < 0 || vIdx >= temp_positions.size()) {
                        LOG_WARN("OBJ face index out of range (v = %d)", vIdx);
                        continue;
                    }

					// OBJ indices are 1-based, so we subtract 1 to convert to 0-based
                    glm::vec3 pos = temp_positions[vIdx];
                    glm::vec3 normal = (vnIdx >= 0 && vnIdx < temp_normals.size()) ? temp_normals[vnIdx] : glm::vec3(0);
                    glm::vec2 uv = (vtIdx >= 0 && vtIdx < temp_uvs.size()) ? temp_uvs[vtIdx] : glm::vec2(0);

                    // Push vertex
                    pMesh->_vertices.emplace_back(pos, normal, uv);
                    // Push index
                    pMesh->_indices.push_back((unsigned int)pMesh->_vertices.size() - 1);
                }
            }
        }

        //LOG_INFO("OBJ import finished. Vertices: %zu  Indices: %zu", pMesh->_vertices.size(), pMesh->_indices.size());
        return true;
    }
}