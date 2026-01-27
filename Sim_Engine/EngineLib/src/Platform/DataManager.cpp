#include "pch.h"
#include "Platform/DataManager.h"

namespace data {
	// Escape a string for CSV format
    inline std::string escape_csv(const std::string_view s) {
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

	inline std::string toString(const Value& v) {
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
			std::string operator()(const std::string& s) const { return s; }
		} visitor;
		return std::visit(visitor, v);
	}

	inline std::string timestampCompact() {
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


	// --- CsvStreamWriter Implementation ---

	void CsvStreamWriter::start(std::string_view parentFolder, std::string_view subFolder) {
		std::lock_guard<std::mutex> lock(_mtx);
		if (_active) return;

		// Create folder if it doesn't exist
		std::filesystem::path dir = std::filesystem::path(parentFolder) / subFolder;
		std::error_code ec;
		std::filesystem::create_directories(dir, ec);
		if (ec) {
			std::cerr << "DataManager: create_directories failed: " << ec.message()
				<< " dir=" << dir.string() << "\n";
		}

		_path = (dir / ("dsfe_run_" + timestampCompact() + ".csv")).string();
		_file.open(_path, std::ios::out);

		if (!_file.is_open()) {
			std::cerr << "DataManager: failed to open CSV file for writing: " << _path
				<<  "cwd=" << std::filesystem::current_path().string() << "\n";
			_active = false;
			return;
		}

		_active = true;
		_file << "topic,key,value\n"; // CSV header (Im bad at remembering to add docs, so here incase I roll this program wider)
		_file.flush(); // ensure header is written
	}

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


	// --- DataManager Implementation ---

	void DataManager::setEnabled(bool enabled) {
		if (enabled == _enabled) return;
		_enabled = enabled;

		if (_enabled) {
			_sim.start(_parentFolder, "Simulation");
			_ref.start(_parentFolder, "Reference");
		}
		else {
			_sim.stop();
			_ref.stop();
		}
	}

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