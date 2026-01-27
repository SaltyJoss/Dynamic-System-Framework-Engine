#pragma once

#include "EngineCore.h"
#include <string>
#include <string_view>
#include <vector>
#include <utility>
#include <variant>
#include <fstream>
#include <mutex>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <iostream>

namespace data {
	// Variant type to hold different data types
    using Value = std::variant<
        std::nullptr_t,
        bool,
        int64_t,
        uint64_t,
        double,
        float,
        long,
        std::string
    >;
    
	// Field type representing a key-value pair
	using Field = std::pair<std::string_view, Value>;
	// List of fields
	using FieldList = std::vector<Field>;

    enum class
}