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
        long,
        std::string
    >;
    
	// Field type representing a key-value pair
	using Field = std::pair<std::string_view, Value>;
	// List of fields
	using FieldList = std::vector<Field>;

	// Stream enum for data streams
    enum class Stream { Simulation, Reference };

    class ENGINE_API CsvStreamWriter {
    public: 
		CsvStreamWriter() = default;

        void start(std::string_view parentFolder, std::string_view subFolder);
		void stop();

		bool active() const { return _active; }
		const std::string& path() const { return _path; }

		void write(std::string topic, const FieldList& fields);

    private:
        mutable std::mutex _mtx;
        std::ofstream _file;
        std::string _path;
        bool _active = false;
    };

    class ENGINE_API DataManager {
    public:
		// Singleton instance accessor
        static DataManager& instance() {
            static DataManager instance;
            return instance;
		}

		// Enable or disable data logging
        void setEnabled(bool enabled);

		// Set parent folder for data logging
        void setParentFolder(std::string folder) { _parentFolder = folder; }

		// Start data logging session
		void capture(Stream s, std::string_view topic, const FieldList& fields);

		// Check if data logging is enabled
		bool enabled() const { return _enabled; }

    private:
        DataManager() = default;

        bool _enabled = false;
        std::string _parentFolder = "Runs";
        CsvStreamWriter _sim;
        CsvStreamWriter _ref;
    };
} // namespace data

// ---macro definitions ---

// DATA_CAPTURE_ENABLE
#ifdef CAPTURE_SIM_DATA
#error CAPTURE_SIM_DATA already defined before DataManager.h
#endif
// Enable or disable data capture
#define DATA_CAPTURE_ENABLE(b) \
    do { ::data::DataManager::instance().setEnabled((b)); } while(0)

// CAPTURE_SIM_DATA
#ifdef CAPTURE_SIM_DATA
#error CAPTURE_SIM_DATA already defined before DataManager.h
#endif
// Capture simulation data
#define CAPTURE_SIM_DATA(topic, fields) \
    do { ::data::DataManager::instance().capture(::data::Stream::Simulation, (topic), (fields)); } while(0)

// CAPTURE_REF_DATA
#ifdef CAPTURE_REF_DATA
#error CAPTURE_REF_DATA already defined before DataManager.h
#endif
// Capture reference data
#define CAPTURE_REF_DATA(topic, fields) \
    do { ::data::DataManager::instance().capture(::data::Stream::Reference, (topic), (fields)); } while(0)

