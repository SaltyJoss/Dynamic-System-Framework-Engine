#include "ch.h"
#include "importObj.h"
#include "Elements/VertexHolder.h"
#include "Utils/str_utils.h"

namespace nmesh_import
{
  bool ObjMeshImporter::fromFile(const std::string& filepath, elements::Mesh* pMesh)
  {
    std::ifstream in(filepath, std::ios::in);
    if (!in)
    {
      return false;
    }

    std::vector<glm::vec3> t_vert;

    std::string s_line;
    while (std::getline(in, s_line))
    {
      std::istringstream ss_line(s_line);
      std::string id;
      ss_line >> id;


      if (id == "v")
      {
        // read vertices
        glm::vec3 v;

        ss_line >> v.x >> v.y >> v.z;

        // Add to temporary vertices before indexing
        t_vert.push_back(v);
      }

      // Faces
      else if (id == "f")
      {
        // TODO: Add quads, now we just have tris
        // TODO: read indizes for normals and UVs
        // TODO: Optimize data structure to cache indices (map)
        std::string v1, v2, v3;
        ss_line >> v1 >> v2 >> v3;

        uint32_t vert_idx[3];
        vert_idx[0] = nutils::tokenize(v1, '/').at(0);
        vert_idx[1] = nutils::tokenize(v2, '/').at(0);
        vert_idx[2] = nutils::tokenize(v3, '/').at(0);

        pMesh->addVertexIndex(vert_idx[0] - 1);
        pMesh->addVertexIndex(vert_idx[1] - 1);
        pMesh->addVertexIndex(vert_idx[2] - 1);        
      }
    }

    // Now use the indices to create the concrete vertices for the mesh
    for (auto v_idx : pMesh->getVertexIndices())
    {
      glm::vec3 vertex = t_vert[v_idx];
      //pMesh->add_vertex(vertex);
    }
    return true;
  }
}