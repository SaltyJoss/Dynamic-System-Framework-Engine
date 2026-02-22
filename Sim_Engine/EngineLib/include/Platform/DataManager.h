#pragma once
// File:   DataManager.h
// GitHub: SaltyJoss
#pragma warning(disable : 4251)
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

        void start(std::string_view parentFolder, std::string_view subFolder, std::string intName);
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
		DataManager() = default;
        ~DataManager();

        void finalise();

        void setEnabled(bool enabled);
		void setIntegratorName(std::string name) { _integratorName = name; }
        void setParentFolder(std::string folder) { _parentFolder = folder; }
		void capture(Stream s, std::string_view topic, const FieldList& fields);
		bool enabled() const { return _enabled; }

    private:
        bool _enabled = false;
        std::string _integratorName = "Unknown";
        std::string _parentFolder = "Runs";
        HDF5StreamWriter _sim;
        HDF5StreamWriter _ref;
    };
} // namespace data