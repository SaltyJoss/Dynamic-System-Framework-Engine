// DSFE_GUI RigidBodyRenderer.h
#pragma once

#include <unordered_map>
#include <functional>
#include <vector>
#include <string>

#include <core/Types.h>

namespace scene { class Object; }
namespace systems { struct RigidBodyModel; }

struct LinkRenderData {
	std::vector<scene::Object*> visuals; // Visual objects associated with this link
	std::vector<scene::Object*> collisions; // Collision objects associated with this link (not implemented yet)
};

struct RigidBodyRenderBinding;

class RigidBodyRenderer {
public:
	void bind(const RigidBodyRenderBinding& binding);
	void applyTransforms(const systems::RigidBodyModel& robot, const std::vector<mathlib::Mat4>& world);
	void clearRigidBodyModel(const systems::RigidBodyModel& robot);

private:
	struct linkRenderData {
		std::vector<scene::Object*> visuals; // Visual objects associated with this links
	};

	std::unordered_map<std::string, LinkRenderData> linkRenderMap; // Map from link names to their render data
};