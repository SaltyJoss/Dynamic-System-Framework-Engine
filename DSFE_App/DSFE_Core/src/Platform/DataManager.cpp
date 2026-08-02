/*
 * File: Platform/DataManager.cpp
 * Created by: Joss Salton, 26-07-2026
 */
#include "pch.h"

#include "Platform/DataManager.h"
#include "Analysis/MetricLogger.h"

// HDF5 C API
#include <hdf5.h>

namespace data {
	// Escape a string for CSV format
	static inline std::string escape_csv(const std::string_view s) {
		bool needQuotes = false;
		for (char c : s) {
			if (c == ',' || c == '"' || c == '\n' || c == '\r') {
				needQuotes = true;
				break;
			}
		}

		if (!needQuotes) { return std::string(s); }

		std::string out;
		out.reserve(s.size() + 2);
		out.push_back('"');
		for (char c : s) {
			if (c == '"') { out += "\"\""; } // double the quotes
			else { out.push_back(c); }
		}
		out.push_back('"');
		return out;
	}

	// Convert a Value to a string representation
	static inline std::string toString(const Value& v) {
		// Visitor struct to convert Value to string
		struct {
			std::string operator()(std::nullptr_t) const { return "null"; }
			std::string operator()(bool b) const { return b ? "true" : "false"; }
			std::string operator()(int64_t i) const { return std::to_string(i); }
			std::string operator()(uint64_t u) const { return std::to_string(u); }
			std::string operator()(double d) const {
				std::ostringstream oss;
				oss.setf(std::ios::fixed);
				oss << std::setprecision(10) << d;
				return oss.str();
			}
			std::string operator()(const std::vector<double>& vec) const {
				std::ostringstream oss;
				oss << "[";
				for (size_t i = 0; i < vec.size(); ++i) {
					oss.setf(std::ios::fixed);
					oss << std::setprecision(10) << vec[i];
					if (i + 1 < vec.size()) oss << ", ";
				}
				oss << "]";
				return oss.str();
			}
			std::string operator()(const std::vector<std::string>& vec) const {
				std::ostringstream oss;
				oss << "[";
				for (size_t i = 0; i < vec.size(); ++i) {
					oss << "\"" << vec[i] << "\"";
					if (i + 1 < vec.size()) oss << ", ";
				}
				oss << "]";
				return oss.str();
			}
			std::string operator()(const std::string& s) const { return s; }
		} visitor;
		return std::visit(visitor, v);
	}

	// Get current timestamp as a compact string (e.g. "20240601_153045")
	static inline std::string timestampCompact() {
		auto now = std::chrono::system_clock::now();
		auto secs = std::chrono::time_point_cast<std::chrono::seconds>(now);
		auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - secs).count();
		std::time_t t = std::chrono::system_clock::to_time_t(now);
		std::tm tm{};

		// Thread-safe localtime
#ifdef _WIN32
		localtime_s(&tm, &t);
#else
		localtime_r(&t, &tm);
#endif
		std::ostringstream oss;
		oss << std::put_time(&tm, "%Y%m%d_%H%M%S") << "_" << std::setw(3) << std::setfill('0') << ms;
		return oss.str();
	}

	// --- HDF5 Utility Functions ---

	// RAII closer for HDF5 identifiers
	struct HDF5Closer {
		void operator()(hid_t id) const { if (id >= 0) H5Fclose(id); }
	};

	// Ensure HDF5 group exists, create if not
	static hid_t ensureGroup(hid_t file, const char* path) {
		if (H5Lexists(file, path, H5P_DEFAULT) > 0) {
			return H5Gopen(file, path, H5P_DEFAULT);
		}
		return H5Gcreate(file, path, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
	}

	// Ensure 1D variable-length string dataset with unlimited dimension
	static hid_t ensureVlenStringDataset1D(hid_t file, const std::string& path, hid_t vlenStrType) {
		if (H5Lexists(file, path.c_str(), H5P_DEFAULT) > 0) {
			return H5Dopen(file, path.c_str(), H5P_DEFAULT);
		}

		// Create dataspace with unlimited dimension
		hsize_t dims[1] = { 0 };
		hsize_t maxDims[1] = { H5S_UNLIMITED };
		hid_t space = H5Screate_simple(1, dims, maxDims);

		// Create dataset with chunking
		hid_t dcpl = H5Pcreate(H5P_DATASET_CREATE);
		hsize_t chunkDims[1] = { 1024 };
		H5Pset_chunk(dcpl, 1, chunkDims);

		// Create the dataset
		hid_t ds = H5Dcreate(file, path.c_str(), vlenStrType, space, H5P_DEFAULT, dcpl, H5P_DEFAULT);

		// Cleanup
		H5Pclose(dcpl);
		H5Sclose(space);
		return ds;
	}

	// Ensure 1D double dataset with unlimited dimension
	static hid_t ensureDoubleDataset1D(hid_t file, const std::string& path) {
		// Check if dataset exists
		if (H5Lexists(file, path.c_str(), H5P_DEFAULT) > 0) {
			return H5Dopen(file, path.c_str(), H5P_DEFAULT);
		}

		// Create dataspace with unlimited dimension
		hsize_t dims[1] = { 0 };
		hsize_t maxDims[1] = { H5S_UNLIMITED };
		hid_t space = H5Screate_simple(1, dims, maxDims);

		// Create dataset with chunking
		hid_t dcpl = H5Pcreate(H5P_DATASET_CREATE);
		hsize_t chunkDims[1] = { 1024 };
		H5Pset_chunk(dcpl, 1, chunkDims);

		// Create the dataset
		hid_t dataset = H5Dcreate(file, path.c_str(), H5T_NATIVE_DOUBLE, space, H5P_DEFAULT, dcpl, H5P_DEFAULT);

		// Cleanup
		H5Pclose(dcpl);
		H5Sclose(space);

		return dataset;
	}

	// Ensure 2D double dataset with unlimited first dimension
	static hid_t ensureDoubleDataset2D(hid_t file, const std::string& path, hsize_t dim2) {
		// Check if dataset exists
		if (H5Lexists(file, path.c_str(), H5P_DEFAULT) > 0) {
			return H5Dopen(file, path.c_str(), H5P_DEFAULT);
		}

		// Create dataspace with unlimited first dimension
		hsize_t dims[2] = { 0, dim2 };
		hsize_t maxDims[2] = { H5S_UNLIMITED, dim2 };
		hid_t space = H5Screate_simple(2, dims, maxDims);

		// Create dataset with chunking
		hid_t dcpl = H5Pcreate(H5P_DATASET_CREATE);
		hsize_t chunkDims[2] = { 1024, dim2 };
		H5Pset_chunk(dcpl, 2, chunkDims);

		// Create the dataset
		hid_t dataset = H5Dcreate(file, path.c_str(), H5T_NATIVE_DOUBLE, space, H5P_DEFAULT, dcpl, H5P_DEFAULT);

		// Cleanup
		H5Pclose(dcpl);
		H5Sclose(space);

		return dataset;
	}

	// Ensure 2D variable-length string dataset with unlimited first dimension
	static hid_t ensureVlenStringDataset2D(hid_t file, const std::string& path, hid_t vlenStrType, hsize_t dim2) {
		// Check if dataset exists
		if (H5Lexists(file, path.c_str(), H5P_DEFAULT) > 0) {
			return H5Dopen(file, path.c_str(), H5P_DEFAULT);
		}

		// Create dataspace with unlimited first dimension
		hsize_t dims[2] = { 0, dim2 };
		hsize_t maxDims[2] = { H5S_UNLIMITED, dim2 };
		hid_t space = H5Screate_simple(2, dims, maxDims);

		// Create dataset with chunking
		hid_t dcpl = H5Pcreate(H5P_DATASET_CREATE);
		hsize_t chunkDims[2] = { 1024, dim2 };
		H5Pset_chunk(dcpl, 2, chunkDims);

		// Create the dataset
		hid_t dataset = H5Dcreate(file, path.c_str(), vlenStrType, space, H5P_DEFAULT, dcpl, H5P_DEFAULT);

		// Cleanup
		H5Pclose(dcpl);
		H5Sclose(space);

		return dataset;
	}

	// Append a string to a 1D variable-length string dataset
	static void appendString1D(hid_t dataset, hid_t vlenStrType, const std::string& value) {
		// Get current size
		hid_t currSpace = H5Dget_space(dataset);
		hsize_t dims[1] = { 0 };
		H5Sget_simple_extent_dims(currSpace, dims, nullptr);
		hsize_t currSize = dims[0];
		H5Sclose(currSpace);

		// Extend dataset
		hsize_t newSize = currSize + 1;
		H5Dset_extent(dataset, &newSize);

		// Select hyperslab for new element
		hid_t filespace = H5Dget_space(dataset);
		hsize_t start[1] = { currSize };
		hsize_t count[1] = { 1 };
		H5Sselect_hyperslab(filespace, H5S_SELECT_SET, start, nullptr, count, nullptr);

		// Create memory space
		hid_t memSpace = H5Screate_simple(1, count, nullptr);

		// Write data
		const char* strData = value.c_str();
		H5Dwrite(dataset, vlenStrType, memSpace, filespace, H5P_DEFAULT, &strData);

		// Cleanup
		H5Sclose(filespace);
		H5Sclose(memSpace);
	}

	// Append a double to a 1D double dataset
	static void appendDouble1D(hid_t dataset, double value) {
		// Get current size
		hid_t currSpace = H5Dget_space(dataset);
		hsize_t dims[1] = { 0 };
		H5Sget_simple_extent_dims(currSpace, dims, nullptr);
		hsize_t currSize = dims[0];
		H5Sclose(currSpace);

		// Extend dataset
		hsize_t newSize = currSize + 1;
		H5Dset_extent(dataset, &newSize);

		// Select hyperslab for new element
		hid_t filespace = H5Dget_space(dataset);
		hsize_t start[1] = { currSize };
		hsize_t count[1] = { 1 };
		H5Sselect_hyperslab(filespace, H5S_SELECT_SET, start, nullptr, count, nullptr);

		// Create memory space
		hid_t memSpace = H5Screate_simple(1, count, nullptr);
		H5Dwrite(dataset, H5T_NATIVE_DOUBLE, memSpace, filespace, H5P_DEFAULT, &value);

		// Cleanup
		H5Sclose(filespace);
		H5Sclose(memSpace);
	}

	// Append a row to a 1D vector-of type double- dataset
	static void appendDoubleVector1D(
		hid_t dataset,
		const std::vector<double>& vals
	) {
		if (vals.empty()) { return; }

		hid_t currSpace = H5Dget_space(dataset);
		hsize_t dims[1] = { 0 };

		H5Sget_simple_extent_dims(currSpace, dims, nullptr);
		const hsize_t currSize = dims[0];

		H5Sclose(currSpace);

		const hsize_t newSize = currSize + static_cast<hsize_t>(vals.size());
		H5Dset_extent(dataset, &newSize);

		hid_t filespace = H5Dget_space(dataset);
		hsize_t start[1] = { currSize };
		hsize_t count[1] = { vals.size() };
		H5Sselect_hyperslab(filespace, H5S_SELECT_SET, start, nullptr, count, nullptr);

		hid_t memSpace = H5Screate_simple(1, count, nullptr);
		H5Dwrite(dataset, H5T_NATIVE_DOUBLE, memSpace, filespace, H5P_DEFAULT, vals.data());

		H5Sclose(filespace);
		H5Sclose(memSpace);
	}

	// Append a row to a 2D double dataset
	static void appendDouble2D(hid_t dataset, const std::vector<double>& values) {
		hsize_t dim2 = values.size();
		// Get current size
		hid_t currSpace = H5Dget_space(dataset);
		hsize_t dims[2] = { 0, 0 };
		H5Sget_simple_extent_dims(currSpace, dims, nullptr);
		hsize_t currSize = dims[0];
		hsize_t storedDim2 = dims[1];

		// Cleanup
		H5Sclose(currSpace);
		if (storedDim2 != dim2) { return; }

		// Extend dataset
		hsize_t newSize[2] = { currSize + 1, dim2 };
		H5Dset_extent(dataset, newSize);

		// Select hyperslab for new row
		hid_t filespace = H5Dget_space(dataset);
		hsize_t start[2] = { currSize, 0 };
		hsize_t count[2] = { 1, dim2 };
		H5Sselect_hyperslab(filespace, H5S_SELECT_SET, start, nullptr, count, nullptr);

		// Create memory space
		hid_t memSpace = H5Screate_simple(2, count, nullptr);
		H5Dwrite(dataset, H5T_NATIVE_DOUBLE, memSpace, filespace, H5P_DEFAULT, values.data());

		// Cleanup
		H5Sclose(filespace);
		H5Sclose(memSpace);
	}

	// Append a row to a 2D variable-length string dataset
	static void appendStringRow2D(hid_t dataset, hid_t vlenStrType, const std::vector<std::string>& row) {
		hsize_t dim2 = row.size();

		// Get current size
		hid_t currSpace = H5Dget_space(dataset);
		hsize_t dims[2] = { 0, 0 };
		H5Sget_simple_extent_dims(currSpace, dims, nullptr);
		hsize_t currSize = dims[0];
		hsize_t storedDim2 = dims[1];

		// Cleanup
		H5Sclose(currSpace);
		if (storedDim2 != dim2) { return; }

		// Extend dataset
		hsize_t newSize[2] = { currSize + 1, dim2 };
		H5Dset_extent(dataset, newSize);

		// Select hyperslab for new row
		hid_t filespace = H5Dget_space(dataset);
		hsize_t start[2] = { currSize, 0 };
		hsize_t count[2] = { 1, dim2 };
		H5Sselect_hyperslab(filespace, H5S_SELECT_SET, start, nullptr, count, nullptr);

		// Create memory space
		hid_t memSpace = H5Screate_simple(2, count, nullptr);

		// Prepare data
		std::vector<const char*> strData(dim2);
		for (hsize_t i = 0; i < dim2; ++i) { strData[i] = row[i].c_str(); }
		H5Dwrite(dataset, vlenStrType, memSpace, filespace, H5P_DEFAULT, strData.data());

		// Cleanup
		H5Sclose(filespace);
		H5Sclose(memSpace);
	}

	// --- HDF5StreamWriter Methods ---

	// start HDF5 stream writer
	void HDF5StreamWriter::start(std::string_view parentFolder, std::string_view subFolder, std::string runTag) {
		std::lock_guard<std::mutex> lock(_mtx);
		if (_active) return;

		std::string parentStr(parentFolder);
		std::string subStr(subFolder);

		// Create folder if it doesn't exist
		std::filesystem::path dir = std::filesystem::path(parentStr) / subStr;
		std::error_code ec;
		std::filesystem::create_directories(dir, ec);
		if (ec) {
			std::cerr << "DataManager: create_directories failed: " << ec.message()
				<< " dir=" << dir.string() << "\n";
		}

		// Determine type string based on subfolder
		std::string typeStr;

		// Determine type string based on subfolder
		if (subStr == "Simulation") { typeStr = "sim"; }
		else if (subStr == "Reference") { typeStr = "ref"; }

		// Get thread ID as string
		std::ostringstream tid;
		tid << std::this_thread::get_id();

		// Format file name to run tag
		_path = (dir / (runTag + ".h5")).string();

		// Create HDF5 file
		static std::mutex hdf5Mutex; // protect HDF5 library calls
		{
			std::lock_guard<std::mutex> hdf5Lock(hdf5Mutex);
			_fileID = H5Fcreate(_path.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
		}
		// Check for errors
		if (_fileID < 0) {
			std::cerr << "Hdf5StreamWriter: failed to create file: " << _path << "\n";
			_fileID = -1;
			_active = false;
			return;
		}

		// Create variable-length string datatype
		_vlenStrType = H5Tcopy(H5T_C_S1);
		H5Tset_size(_vlenStrType, H5T_VARIABLE);

		// Create /log group
		hid_t g = ensureGroup(_fileID, "/log");
		if (g >= 0) { H5Gclose(g); }

		// Check for errors
		if (_vlenStrType < 0 || _fileID < 0) {
			std::cerr << "Hdf5StreamWriter: failed to initialise HDF5 writer.\n";

			// Cleanup
			if (_vlenStrType >= 0) { H5Tclose(_vlenStrType); }
			if (_fileID >= 0) { H5Fclose(_fileID); }

			// Reset handles
			_vlenStrType = -1;
			_fileID = -1;
			_active = false;

			return;
		}

		_active = true;
	}

	// stop HDF5 stream writer
	void HDF5StreamWriter::stop() {
		printf("WRITER STOP\n");
		std::lock_guard<std::mutex> lock(_mtx);
		if (!_active) return;

		// Close cached datasets
		for (auto& [path, ds] : _ds1D_D) { if (ds >= 0) H5Dclose(ds); }
		for (auto& [path, ds] : _ds2D_D) { if (ds >= 0) H5Dclose(ds); }
		for (auto& [path, ds] : _ds2D_vlenStr) { if (ds >= 0) H5Dclose(ds); }
		for (auto& [path, ds] : _ds1D_vlenStr) { if (ds >= 0) H5Dclose(ds); }

		// Clear caches
		_ds1D_D.clear();
		_ds2D_D.clear();
		_ds2D_vlenStr.clear();
		_ds1D_vlenStr.clear();

		// Close datatypes and file
		if (_vlenStrType >= 0) { H5Tclose(_vlenStrType); }
		if (_fileID >= 0) {
			H5Fflush(_fileID, H5F_SCOPE_GLOBAL);
			H5Fclose(_fileID);
		}

		// reset
		_vlenStrType = -1;
		_fileID = -1;
		_active = false;
	}

	// Write fields to HDF5 datasets
	void HDF5StreamWriter::write(std::string topic, const FieldList& fields) {
		std::lock_guard<std::mutex> lock(_mtx);
		if (!_active) { return; }

		// Ensure group for topic
		std::string gPath = "/log/" + topic;
		hid_t g = ensureGroup(_fileID, gPath.c_str());
		if (g < 0) { return; }
		H5Gclose(g);

		// Ensure dataset for key
		auto ensureKey = [&](const std::string& key, const Value& val) -> hid_t {
			const std::string dPath = gPath + "/" + key;

			// Scalar numeric types -> 1D double dataset
			if (std::holds_alternative<double>(val) ||
				std::holds_alternative<int64_t>(val) ||
				std::holds_alternative<uint64_t>(val)) {

				if (auto it = _ds1D_D.find(dPath); it != _ds1D_D.end()) { return it->second; }

				hid_t ds = ensureDoubleDataset1D(_fileID, dPath);
				_ds1D_D.emplace(dPath, ds);
				return ds;
			}

			// scalar string -> 1D vlen string
			if (std::holds_alternative<std::string>(val)) {
				if (auto it = _ds1D_vlenStr.find(dPath); it != _ds1D_vlenStr.end()) return it->second;
				hid_t ds = ensureVlenStringDataset1D(_fileID, dPath, _vlenStrType);
				_ds1D_vlenStr.emplace(dPath, ds);
				return ds;
			}

			// vector<double> -> 2D double dataset
			if (std::holds_alternative<std::vector<double>>(val)) {
				const auto& row = std::get<std::vector<double>>(val);
				const hsize_t dim2 = static_cast<hsize_t>(row.size());

				if (auto it = _ds2D_D.find(dPath); it != _ds2D_D.end()) { return it->second; }

				hid_t ds = ensureDoubleDataset2D(_fileID, dPath, dim2);
				_ds2D_D.emplace(dPath, ds);
				return ds;
			}

			// vector<string> -> 2D variable-length string dataset
			if (std::holds_alternative<std::vector<std::string>>(val)) {
				const auto& row = std::get<std::vector<std::string>>(val);
				const hsize_t dim2 = static_cast<hsize_t>(row.size());

				if (auto it = _ds2D_vlenStr.find(dPath); it != _ds2D_vlenStr.end()) { return it->second; }

				hid_t ds = ensureVlenStringDataset2D(_fileID, dPath, _vlenStrType, dim2);
				_ds2D_vlenStr.emplace(dPath, ds);
				return ds;
			}

			return -1; // unsupported
			};

		// Append each field
		for (const auto& [key, value] : fields) {
			hid_t ds = ensureKey(key, value);
			if (ds < 0) { continue; } // unsupported type

			if (std::holds_alternative<std::string>(value)) {
				appendString1D(ds, _vlenStrType, std::get<std::string>(value));
			}
			else if (std::holds_alternative<double>(value)) {
				appendDouble1D(ds, std::get<double>(value));
			}
			else if (std::holds_alternative<int64_t>(value)) {
				appendDouble1D(ds, static_cast<double>(std::get<int64_t>(value)));
			}
			else if (std::holds_alternative<uint64_t>(value)) {
				appendDouble1D(ds, static_cast<double>(std::get<uint64_t>(value)));
			}
			else if (std::holds_alternative<std::vector<double>>(value)) {
				appendDouble2D(ds, std::get<std::vector<double>>(value));
			}
			else if (std::holds_alternative<std::vector<std::string>>(value)) {
				appendStringRow2D(ds, _vlenStrType, std::get<std::vector<std::string>>(value));
			}
		}
	}

	// Writes vector fields to HDF5 datasets
	void HDF5StreamWriter::writeVector(
		const std::string& topic,
		const std::string& key,
		const std::vector<double>& vals
	) {
		std::lock_guard<std::mutex> lock(_mtx);
		if (!_active || vals.empty()) { return; }

		const std::string gPath = "/log/" + topic;
		hid_t g = ensureGroup(_fileID, gPath.c_str());
		if (g < 0) { return; }

		H5Gclose(g);
		const std::string dPath = gPath + "/" + key;

		hid_t ds;

		if (auto it = _ds1D_D.find(dPath); it != _ds1D_D.end()) {
			ds = it->second;
		}
		else {
			ds = ensureDoubleDataset1D(_fileID, dPath);
			_ds1D_D.emplace(dPath, ds);
		}

		appendDoubleVector1D(ds, vals);
	}

	// start CSV stream writer
	void CsvStreamWriter::start(std::string_view parentFolder, std::string_view subFolder) {
		std::lock_guard<std::mutex> lock(_mtx);
		if (_active) return;

		std::string parentStr(parentFolder);
		std::string subStr(subFolder);

		// Create folder if it doesn't exist
		std::filesystem::path dir = std::filesystem::path(parentStr) / subStr;
		std::error_code ec;
		std::filesystem::create_directories(dir, ec);
		if (ec) {
			std::cerr << "DataManager: create_directories failed: " << ec.message()
				<< " dir=" << dir.string() << "\n";
		}
		
		std::string typeStr;

		if (subStr == "Simulation") { typeStr = "sim"; }
		else if (subStr == "Reference") { typeStr = "ref"; }

		_path = (dir / ("dsfe_" + typeStr + "_run_" + timestampCompact() + ".csv")).string();
		_file.open(_path, std::ios::out);

		if (!_file.is_open()) {
			std::cerr << "DataManager: failed to open CSV file for writing: " << _path
				<< "cwd=" << std::filesystem::current_path().string() << "\n";
			_active = false;
			return;
		}

		_active = true;
		_file << "topic,key,value\n"; // CSV header (Im bad at remembering to add docs, so here incase I roll this program wider)
		_file.flush(); // ensure header is written
	}

	// stop CSV stream writer
	void CsvStreamWriter::stop() {
		std::lock_guard<std::mutex> lock(_mtx);
		if (_file.is_open()) {
			_file.flush();
			_file.close();
		}
		_active = false;
	}

	// Write fields to CSV file
	void CsvStreamWriter::write(std::string topic, const FieldList& fields) {
		std::lock_guard<std::mutex> lock(_mtx);
		if (!_active || !_file.is_open()) return;
		for (const auto& [key, value] : fields) {
			std::string val = toString(value);
			_file << escape_csv(topic) << ","
				<< escape_csv(key) << ","
				<< escape_csv(toString(val)) << "\n";
		}
		_file.flush(); // ensure data is written
	}

	DataManager::~DataManager() {
		printf("DM DESTROYED\n");
		finalise();
	}

	void DataManager::finalise() {
		if (_sim.active()) { _sim.stop(); }
		if (_ref.active()) { _ref.stop(); }
	}

	// Enable or disable data logging
	void DataManager::setEnabled(bool enabled) {
		if (enabled == _enabled) return;
		_enabled = enabled;
		std::string intName = _integratorName.empty() ? "unknown" : _integratorName;
		std::string runTag = _runTag.empty() ? (intName + "_" + timestampCompact()) : _runTag;

		if (_enabled) {
			_sim.start(_parentFolder, "Simulation", runTag);
			_ref.start(_parentFolder, "Reference", runTag);
		}
		else {
			_sim.stop();
			_ref.stop();
		}
	}

	// Capture data to the appropriate stream
	void DataManager::capture(Stream s, std::string_view topic, const FieldList& fields) {
		if (!_enabled) return;
		if (s == Stream::Simulation) {
			_sim.write(std::string(topic), fields);
		}
		else if (s == Stream::Reference) {
			_ref.write(std::string(topic), fields);
		}
	}

	void DataManager::captureJointBuffer(
		Stream s,
		std::string_view topic,
		const systems::JointLogBuffer& buf
	) {
		HDF5StreamWriter* writer = nullptr;
		if (s == Stream::Simulation) { writer = &_sim; }
		else if (s == Stream::Reference) { writer = &_ref; }
		if (!writer) { return; }
		std::vector<double> jointIndexD(buf.joint_index.begin(), buf.joint_index.end());
		const std::string t(topic);
		// Time and step info
		writer->writeVector(t, "sim_time", buf.sim_time);
		writer->writeVector(t, "dt_taken", buf.dt_taken);
		writer->writeVector(t, "dt_sug", buf.dt_sug);
		for (size_t i = 0; i < buf.joint_name.size(); ++i) {
			// States
			writer->writeVector(t + "/" + buf.joint_name[i], "position", std::vector<double>{buf.theta[i]});
			writer->writeVector(t + "/" + buf.joint_name[i], "velocity", std::vector<double>{buf.omega[i]});
			writer->writeVector(t + "/" + buf.joint_name[i], "acceleration", std::vector<double>{buf.alpha[i]});
			writer->writeVector(t + "/" + buf.joint_name[i], "error", std::vector<double>{buf.err[i]});
			writer->writeVector(t + "/" + buf.joint_name[i], "error_d", std::vector<double>{buf.err_d[i]});
			// Dynamics
			writer->writeVector(t + "/" + buf.joint_name[i], "I_eff", std::vector<double>{buf.I_eff[i]});
			writer->writeVector(t + "/" + buf.joint_name[i], "tau", std::vector<double>{buf.tau[i]});
			writer->writeVector(t + "/" + buf.joint_name[i], "tau_ff", std::vector<double>{buf.tau_ff[i]});
			writer->writeVector(t + "/" + buf.joint_name[i], "tau_gravity", std::vector<double>{buf.tau_gravity[i]});
			writer->writeVector(t + "/" + buf.joint_name[i], "tau_barrier", std::vector<double>{buf.tau_barrier[i]});
			writer->writeVector(t + "/" + buf.joint_name[i], "tau_sat", std::vector<double>{buf.tau_sat[i]});
			// Energy, Work, & Power
			writer->writeVector(t + "/" + buf.joint_name[i], "KE", std::vector<double>{buf.KE[i]});
			writer->writeVector(t + "/" + buf.joint_name[i], "PE", std::vector<double>{buf.PE[i]});
			writer->writeVector(t + "/" + buf.joint_name[i], "E_total", std::vector<double>{buf.E_total[i]});
			// Limit flags and info
			writer->writeVector(t + "/" + buf.joint_name[i], "clamp_theta", std::vector<double>{buf.clamp_theta[i]});
			writer->writeVector(t + "/" + buf.joint_name[i], "clamp_omega", std::vector<double>{buf.clamp_omega[i]});
			writer->writeVector(t + "/" + buf.joint_name[i], "sat_flag", std::vector<double>{buf.sat_flag[i]});
		}
	}

	void DataManager::captureFreeBodyBuffer(
		Stream s,
		std::string_view topic,
		const systems::FreeBodyLogBuffer& buf
	) {
		HDF5StreamWriter* writer = nullptr;
		if (s == Stream::Simulation) { writer = &_sim; }
		else if (s == Stream::Reference) { writer = &_ref; }
		if (!writer) { return; }
		std::vector<double> bodyIdxD(buf.body_index.begin(), buf.body_index.end());
		const std::string t(topic);
		// Time and step info
		writer->writeVector(t, "sim_time", buf.sim_time);
		writer->writeVector(t, "dt_taken", buf.dt_taken);
		writer->writeVector(t, "dt_sug", buf.dt_sug);
		for (size_t i = 0; i < buf.body_name.size(); ++i) {
			// States
			writer->writeVector(t + "/" + buf.body_name[i] + "/position", "pos_x", std::vector<double>{buf.pos_x[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/position", "pos_y", std::vector<double>{buf.pos_y[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/position", "pos_z", std::vector<double>{buf.pos_z[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/orientation", "quat_w", std::vector<double>{buf.quat_w[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/orientation", "quat_x", std::vector<double>{buf.quat_x[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/orientation", "quat_y", std::vector<double>{buf.quat_y[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/orientation", "quat_z", std::vector<double>{buf.quat_z[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/velocity", "lin_vel_x", std::vector<double>{buf.linVel_x[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/velocity", "lin_vel_y", std::vector<double>{buf.linVel_y[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/velocity", "lin_vel_z", std::vector<double>{buf.linVel_z[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/velocity", "ang_vel_x", std::vector<double>{buf.angVel_x[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/velocity", "ang_vel_y", std::vector<double>{buf.angVel_y[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/velocity", "ang_vel_z", std::vector<double>{buf.angVel_z[i]});
			// Time Derivatives
			writer->writeVector(t + "/" + buf.body_name[i] + "/acceleration", "lin_acc_x", std::vector<double>{buf.linAcc_x[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/acceleration", "lin_acc_y", std::vector<double>{buf.linAcc_y[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/acceleration", "lin_acc_z", std::vector<double>{buf.linAcc_z[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/acceleration", "ang_acc_x", std::vector<double>{buf.angAcc_x[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/acceleration", "ang_acc_y", std::vector<double>{buf.angAcc_y[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/acceleration", "ang_acc_z", std::vector<double>{buf.angAcc_z[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/F_net", "F_net_x", std::vector<double>{buf.F_net_x[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/F_net", "F_net_y", std::vector<double>{buf.F_net_y[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/F_net", "F_net_z", std::vector<double>{buf.F_net_z[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/tau_net", "tau_net_x", std::vector<double>{buf.tau_net_x[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/tau_net", "tau_net_y", std::vector<double>{buf.tau_net_y[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/tau_net", "tau_net_z", std::vector<double>{buf.tau_net_z[i]});
			// Energy & Performance
			writer->writeVector(t + "/" + buf.body_name[i], "KE", std::vector<double>{buf.KE[i]});
			writer->writeVector(t + "/" + buf.body_name[i], "PE", std::vector<double>{buf.PE[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/momentum", "lin_mom_x", std::vector<double>{buf.linMom_x[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/momentum", "lin_mom_y", std::vector<double>{buf.linMom_y[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/momentum", "lin_mom_z", std::vector<double>{buf.linMom_z[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/momentum", "ang_mom_x", std::vector<double>{buf.angMom_x[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/momentum", "ang_mom_y", std::vector<double>{buf.angMom_y[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/momentum", "ang_mom_z", std::vector<double>{buf.angMom_z[i]});
			// Sleep State
			writer->writeVector(t + "/" + buf.body_name[i], "sleep_state", std::vector<double>{buf.sleep_state[i]});
			// Mass & Inertia
			writer->writeVector(t + "/" + buf.body_name[i], "mass", std::vector<double>{buf.mass[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/inertia", "I_xx", std::vector<double>{buf.Ixx[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/inertia", "I_yy", std::vector<double>{buf.Iyy[i]});
			writer->writeVector(t + "/" + buf.body_name[i] + "/inertia", "I_zz", std::vector<double>{buf.Izz[i]});
		}
	}

} // namespace data