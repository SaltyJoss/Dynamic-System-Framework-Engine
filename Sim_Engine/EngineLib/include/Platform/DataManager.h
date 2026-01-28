#pragma once

#include "EngineCore.h"
#include <string>
#include <string_view>
#include <vector>
#include <unordered_map>
#include <utility>
#include <variant>
#include <fstream>
#include <mutex>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <iostream>

// HDF5 C API
#include <hdf5.h>

namespace data {
	// Variant type to hold different data types
    using Value = std::variant<
        std::nullptr_t, bool, int64_t, uint64_t, double, long,
        std::vector<double>, 
        std::vector<std::string>, std::string
    >;
    
	// Field type representing a key-value pair
	using Field = std::pair<std::string, Value>;
	// List of fields
	using FieldList = std::vector<Field>;

	// Stream enum for data streams
    enum class Stream { Simulation, Reference };

    class ENGINE_API HDF5StreamWriter {
	public:
		HDF5StreamWriter() = default;

		void start(std::string_view parentFolder, std::string_view subFolder);
		void stop();

		bool active() const { return _active; }
		const std::string& path() const { return _path; }

		void write(std::string topic, const FieldList& fields);

    private:
        mutable std::mutex _mtx;
        std::string _path;
        bool _active = false;

		// HDF5 file and datatype handles 
        hid_t _fileID = -1;
        hid_t _vlenStrType = -1;
        
		// Cached dataset handles for 1D data
		std::unordered_map<std::string, hid_t> _ds1D_D;        // 1D scalar numeric rows (double)
        std::unordered_map<std::string, hid_t> _ds1D_vlenStr;  // 1D string scalars  <-- ADD THIS
		// Cached dataset handles for 2D data
        std::unordered_map<std::string, hid_t> _ds2D_D;        // 2D numeric rows (vector<double>)
        std::unordered_map<std::string, hid_t> _ds2D_vlenStr;  // 2D string rows (vector<string>)

    };

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

        std::vector<std::string> _header;               // column names
        std::unordered_map<std::string, size_t> _colIx; // name -> index
        bool _wroteHeader = false;

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
        HDF5StreamWriter _sim;
        HDF5StreamWriter _ref;
    };
} // namespace data

// ---macro definitions ---

// DATA_CAPTURE_ENABLE
#ifdef DATA_CAPTURE_ENABLE
#error DATA_CAPTURE_ENABLE already defined before DataManager.h
#endif
// Enable or disable data capture
#define DATA_CAPTURE_ENABLE(b) \
    do { ::data::DataManager::instance().setEnabled((b)); } while(0)

// --- HDF5 Macros ---

// HDF5_SIM_DATA
#ifdef HDF5_SIM_DATA
#error HDF5_SIM_DATA already defined before DataManager.h
#endif
// Capture simulation data as HDF5
#define HDF5_SIM_DATA(topic, fields) \
    do { ::data::DataManager::instance().capture(::data::Stream::Simulation, (topic), (fields)); } while(0)

// HDF5_REF_DATA
#ifdef HDF5_REF_DATA
#error HDF5_REF_DATA already defined before DataManager.h
#endif
// Capture reference data as HDF5
#define HDF5_REF_DATA(topic, fields) \
    do { ::data::DataManager::instance().capture(::data::Stream::Reference, (topic), (fields)); } while(0)

// --- CSV Macros ---

// CSV_SIM_DATA
#ifdef CSV_SIM_DATA
#error CSV_SIM_DATA already defined before DataManager.h
#endif
// Capture simulation data as CSV
#define CSV_SIM_DATA(topic, fields) \
    do { ::data::DataManager::instance().capture(::data::Stream::Simulation, (topic), (fields)); } while(0)

// CSV_REF_DATA
#ifdef CSV_REF_DATA
#error CSV_REF_DATA already defined before DataManager.h
#endif
// Capture reference data as CSV
#define CSV_REF_DATA(topic, fields) \
    do { ::data::DataManager::instance().capture(::data::Stream::Reference, (topic), (fields)); } while(0)


// --- end of macro definitions ---
