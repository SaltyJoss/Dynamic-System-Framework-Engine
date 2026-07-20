// DSFE_GUI Simulation/SimulationScene.cpp
#include "Simulation/SimulationScene.h"

namespace gui {
    uint32_t SimulationScene::add_renderable(uint32_t mesh_id, const glm::mat4& transform) {
        Renderable r{ mesh_id, transform };
        _renderables.push_back(r);
        return static_cast<uint32_t>(_renderables.size() - 1);
    }

    void SimulationScene::set_transform(uint32_t idx, const glm::mat4& transform) {
        if (idx < _renderables.size()) { _renderables[idx].transform = transform; }
    }
} // namespace gui