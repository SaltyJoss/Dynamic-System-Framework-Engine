#pragma once
// File:   str_utils.h
// GitHub: SaltyJoss
// -----
// Initially templated off a tutorial:
// GitHub: jayanam/jgl_demos/JGL_MeshLoader
#include "EngineCore.h"
#include "Platform/Logger.h"

namespace utils {
  std::vector<uint32_t> tokenize(const std::string& line, const char token) {
    std::vector<uint32_t> result;
    
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, token))
      if (!item.empty())
        result.push_back(std::stoi(item));

    return result;
  }
} // namespace utils
