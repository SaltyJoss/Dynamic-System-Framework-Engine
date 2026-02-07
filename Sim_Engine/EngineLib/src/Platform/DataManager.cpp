#include "pch.h"
#include "Platform/DataManager.h"

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
			std::string operator()(long l) const {
				std::ostringstream oss;
				oss.setf(std::ios::fixed);
				oss << std::setprecision(10) << l;
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

	static inline std::string timestampCompact() {
		auto now = std::chrono::system_clock::now();
		std::time_t t = std::chrono::system_clock::to_time_t(now);
		std::tm tm{};

		// Thread-safe localtime
#ifdef _WIN32
		localtime_s(&tm, &t);
#else
		localtime_r(&t, &tm);
#endif
		std::ostringstream oss;
		oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
		return oss.str();
	}

	// =======================================
	// --- HDF5StreamWriter Implementation ---
	// =======================================

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
	void HDF5StreamWriter::start(std::string_view parentFolder, std::string_view subFolder, std::string intName) {
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

		_path = (dir / ("dsfe_" + typeStr + "_run_" + timestampCompact() + "_" + intName + ".h5")).string();

		// Create HDF5 file
		_fileID = H5Fcreate(_path.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
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
				std::holds_alternative<uint64_t>(val) ||
				std::holds_alternative<long>(val)) {

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

			// Append value based on type
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
			else if (std::holds_alternative<long>(value)) {
				appendDouble1D(ds, static_cast<double>(std::get<long>(value)));
			}
			else if (std::holds_alternative<std::vector<double>>(value)) {
				appendDouble2D(ds, std::get<std::vector<double>>(value));
			}
			else if (std::holds_alternative<std::vector<std::string>>(value)) {
				appendStringRow2D(ds, _vlenStrType, std::get<std::vector<std::string>>(value));
			}
		}
	}

	// =======================================
	// --- CsvStreamWriter Implementation ---
	// =======================================

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

	// =======================================
	// --- DataManager Implementation ---
	// =======================================

	// Enable or disable data logging
	void DataManager::setEnabled(bool enabled) {
		if (enabled == _enabled) return;
		_enabled = enabled;
		std::string intName = _integratorName.empty() ? "unknown" : _integratorName;

		if (_enabled) {
			_sim.start(_parentFolder, "Simulation", intName);
			_ref.start(_parentFolder, "Reference", intName);
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

} // namespace data