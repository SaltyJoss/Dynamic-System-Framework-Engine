// DSFE_GUI Simulation/MeshStore.cpp
#include "Simulation/MeshStore.h"

namespace gui {
    uint32_t MeshStore::add(scene::Mesh mesh) {
        _meshes.push_back(std::move(mesh));
        return static_cast<uint32_t>(_meshes.size() - 1);
    }
    const scene::Mesh* MeshStore::get(uint32_t id) const { return id < _meshes.size() ? &_meshes[id] : nullptr; }
} // namespace gui