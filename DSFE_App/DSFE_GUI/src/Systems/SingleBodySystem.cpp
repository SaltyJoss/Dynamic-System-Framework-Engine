// DSFE_GUI Systems/SingleBodySystem.cpp
#include "Systems/SingleBodySystem.h"
#include "SingleBodySystem/Body.h"
#include "Simulation/SimulationScene.h"
#include "Simulation/MeshStore.h"
#include "Simulation/SimulationRenderer.h"

#include "Assets/MeshLoader.h"
#include "Scene/Mesh.h"
#include "SingleBodySystem/Body.h"
#include "EngineLib/LogMacros.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

namespace gui {
    static glm::mat4 toGlm(const mathlib::Mat4& m) {    
        glm::mat4 g(1.0f);
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r)
                g[c][r] = static_cast<float>(m(r, c));
        return g;
    }

    SingleBodySystem::SingleBodySystem(const std::string& mesh_path,single_body_system::SingleBodySystem& core_body,
        float scale, MeshStore& mesh_store, SimulationRenderer& renderer)
        : _mesh_path(mesh_path), _core_body(core_body), _scale(scale), _meshStore(mesh_store), _renderer(renderer) {}

    void SingleBodySystem::build(SimulationScene& scene) {
        assets::MeshLoader loader;
        auto meshes = loader.load(_mesh_path);
        if (meshes.empty() || meshes.front()->_vertices.empty()) {
            LOG_ERROR("No geometry found in mesh file: %s", _mesh_path.c_str());
            return;
        }

        scene::Mesh mesh = *meshes.front();
        std::vector<uint32_t> indices(mesh._indices.begin(), mesh._indices.end());
        const uint32_t cpu_id = _meshStore.add(mesh);
        const uint32_t gpu_id = _renderer.upload(_meshStore.get(cpu_id)->_vertices, indices);

        if (cpu_id != gpu_id) { LOG_ERROR("ID mismatch %u vs %u for mesh: %s", cpu_id, gpu_id, _mesh_path.c_str()); }

        _renderable = scene.add_renderable(cpu_id, glm::mat4(1.0f)); // at origin for now
    }

    void SingleBodySystem::update(SimulationScene& scene) {
        if (_renderable == UINT32_MAX || !_core_body.hasBody()) { return; }
        const single_body_system::Body* body = _core_body.body();

        glm::vec3 pos(
            static_cast<float>(body->state.p.x()),
            static_cast<float>(body->state.p.y()),
            static_cast<float>(body->state.p.z())
        );
        glm::quat rot(
            static_cast<float>(body->state.q.w()),
            static_cast<float>(body->state.q.x()),
            static_cast<float>(body->state.q.y()),
            static_cast<float>(body->state.q.z())
        );

        const glm::mat4 M = glm::translate(glm::mat4(1.0f), pos) * glm::mat4_cast(rot) * glm::scale(glm::mat4(1.0f), glm::vec3(_scale));
        scene.set_transform(_renderable, M);    
    }

    void SingleBodySystem::clear(SimulationScene& scene) {
        _renderable = UINT32_MAX;
    }
} // namespace gui