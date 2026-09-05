/*
 * File: Systems/MultiBodySystem.cpp
 * Created by: Joss Salton, 26-07-2026
 */
#include "Systems/MultiBodySystem.h"
#include "Simulation/SimulationScene.h"
#include "Simulation/MeshStore.h"
#include "Simulation/SimulationRenderer.h"

#include <collision/convex_hull.h>
#include <collision/convex_hull_build.h>

#include "Assets/MeshLoader.h"
#include "Scene/Mesh.h"
#include "Systems/RigidBodyModel.h"
#include "Platform/Paths.h"
#include "EngineLib/LogMacros.h"

#include <glm/gtc/matrix_transform.hpp>
#include <filesystem>
#include "EngineLib/LogMacros.h"

namespace gui {
    static glm::mat4 toGlm(const mathlib::Mat4& m) {
        glm::mat4 g(1.0f);
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r)
                g[c][r] = static_cast<float>(m(r, c));
        return g;
    }

    MultiBodySystem::MultiBodySystem(const systems::RigidBodyModel& model, std::function<const std::vector<mathlib::Mat4>&()> world_src, MeshStore& mesh_store, SimulationRenderer& renderer)
        : _model(model), _world_src(world_src), _meshStore(mesh_store), _renderer(renderer) {}

    void MultiBodySystem::build(SimulationScene& scene) {
        assets::MeshLoader loader;
        namespace fs = std::filesystem;
        for (const auto& link : _model.links) {
            auto& renderables = _binding.link_to_renderables[link.name];
            glm::vec3 lo(1e30f), hi(-1e30f);
            bool anyVerts = false;
            std::vector<mathlib::Vec3> hull_verts;
            for (const auto& entry : link.visual.meshEntries) {
                fs::path full = paths::assets() / entry.meshFile;
                auto meshes = loader.load(full.string());
                if (meshes.empty()) { LOG_ERROR("No meshes in %s", full.string().c_str()); continue; }
                for (auto& mptr : meshes) {
                    scene::Mesh& src = *mptr;
                    if (src._vertices.empty()) { continue; }
                    const float sc = static_cast<float>(_model.scale);
                    for (const auto& v : src._vertices) {
                        lo = glm::min(lo, glm::vec3(v._pos.x, v._pos.y, v._pos.z));
                        hi = glm::max(hi, glm::vec3(v._pos.x, v._pos.y, v._pos.z));
                        anyVerts = true;
                        hull_verts.emplace_back(v._pos.x*sc, v._pos.y*sc, v._pos.z*sc);
                    }
                    std::vector<uint32_t> indices(src._indices.begin(), src._indices.end());
                    const uint32_t cpu_id = _meshStore.add(src);
                    const uint32_t gpu_id = _renderer.upload(_meshStore.get(cpu_id)->_vertices, indices);
                    if (cpu_id != gpu_id) { LOG_ERROR("ID mismatch %u vs %u", cpu_id, gpu_id); }

                    const uint32_t r_idx = scene.add_renderable(cpu_id, glm::mat4(1.0f));
                    if (entry.hasMaterial) {
                        scene.set_material(r_idx, glm::vec3(
                            static_cast<float>(entry.material.x()), 
                            static_cast<float>(entry.material.y()), 
                            static_cast<float>(entry.material.z())), 
                            entry.metallic, entry.roughness, 1.0f
                        );
                    }
                    renderables.push_back(r_idx);
                }
            }
            if (anyVerts) {
                auto& L = const_cast<systems::RigidBodyLink&>(link);
                const float s = _model.scale;
                L.aabbMin = mathlib::Vec3(lo.x * s, lo.y * s, lo.z * s);
                L.aabbMax = mathlib::Vec3(hi.x * s, hi.y * s, hi.z * s);
                L.hasBounds = true;
                // Build a convex hull for collision detection from the visual mesh vertices
                L.collision.type = systems::eCollisionShape::MESH;
                L.collision.hull = std::make_shared<physlib::collision::ConvexHull>(physlib::collision::buildHull(hull_verts));
                LOG_INFO("Link %s: built collision hull (%zu verts) from %zu visual mesh verts", L.name.c_str(), L.collision.hull->verts.size(), hull_verts.size());

                // if (L.collision.type == systems::eCollisionShape::NONE || L.collision.type == systems::eCollisionShape::MESH) {
                //     L.collision.type = systems::eCollisionShape::MESH;
                //     L.collision.hull = std::make_shared<physlib::collision::ConvexHull>(physlib::collision::buildHull(hull_verts));
                //     LOG_INFO("Link %s: built collision hull (%zu verts) from %zu visual mesh verts", L.name.c_str(), L.collision.hull->verts.size(), hull_verts.size());
                // }
            }
        }
    }

    void MultiBodySystem::update(SimulationScene& scene) {
        const std::vector<mathlib::Mat4>& world = _world_src();
        if (world.size() < _model.links.size()) {
            LOG_ERROR("World transforms size (%zu) less than link count (%zu)", world.size(), _model.links.size());
            return;
        }
        const glm::mat4 scale_M = glm::scale(glm::mat4(1.0f), glm::vec3(_model.scale));
        for (size_t i = 0; i < _model.links.size(); ++i) {
            const auto& link = _model.links[i];
            auto it = _binding.link_to_renderables.find(link.name);
            if (it == _binding.link_to_renderables.end()) { continue; }
            glm::mat4 T = toGlm(world[i]); glm::mat4 M = T * scale_M;
            for (uint32_t r_idx : it->second) { scene.set_transform(r_idx, M); }
        }
    }

    void MultiBodySystem::clear(SimulationScene& scene) {
        // Reset all renderables associated with the body links to identity transforms and clear the binding map
        for (const auto& [link_name, renderables] : _binding.link_to_renderables) {
            for (uint32_t r_idx : renderables) { scene.set_transform(r_idx, glm::mat4(1.0f)); }
        }
        _binding.clear();
    }

} // namespace gui