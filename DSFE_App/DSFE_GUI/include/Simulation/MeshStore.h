// DSFE_GUI Simulation/MeshStore.h
#pragma once

#include "Scene/Mesh.h"

#include <vector>
#include <cstdint>

namespace gui {
    class MeshStore {
        public:
            static constexpr uint32_t  INVALID_ID = UINT32_MAX;

            uint32_t add(scene::Mesh mesh);
            const scene::Mesh* get(uint32_t id) const;
            uint32_t count() const { return static_cast<uint32_t>(_meshes.size()); }
            void clear() { _meshes.clear(); }
            
        private:
            std::vector<scene::Mesh> _meshes; // Internal storage for meshes
    };
} // namespace gui