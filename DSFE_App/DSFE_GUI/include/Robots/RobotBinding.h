// DSFE_GUI Robots/RobotBinding.h
#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <cstdint>

namespace gui {
    struct RobotBinding {
        std::unordered_map<std::string, std::vector<uint32_t>> link_to_renderables;
        void clear() { link_to_renderables.clear(); }
    };
} // namespace gui