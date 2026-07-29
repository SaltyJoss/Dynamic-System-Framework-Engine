// DSFE_GUI SystemMap.h
#pragma once

#include <unordered_map>
#include <string>

namespace platform {
	enum class eRoboticSystems {
		Z1 = 0,
		UR5e = 1,
		Panda = 2,
		iiwa14 = 3,
		VISPA = 4,
		H1 = 5,
		Cube = 6
	};

	enum class eRoboticSystemFamilies {
		Unitree = 0,
		Universal = 1,
		Franka = 2,
		KUKA = 3,
		Airbus = 4,
		Other = 5
	};

	struct RoboticSystems {
		std::unordered_map<eRoboticSystems, eRoboticSystemFamilies> robotMap = {
			{ eRoboticSystems::Z1, eRoboticSystemFamilies::Unitree },
			{ eRoboticSystems::UR5e, eRoboticSystemFamilies::Universal },
			{ eRoboticSystems::Panda, eRoboticSystemFamilies::Franka },
			{ eRoboticSystems::iiwa14, eRoboticSystemFamilies::KUKA },
			{ eRoboticSystems::VISPA, eRoboticSystemFamilies::Airbus },
			{ eRoboticSystems::H1, eRoboticSystemFamilies::Unitree },
			{ eRoboticSystems::Cube, eRoboticSystemFamilies::Other }
		};

		inline std::string toString(eRoboticSystems sys) {
			switch (sys) {
				case eRoboticSystems::Z1: return "Z1";
				case eRoboticSystems::UR5e: return "UR5e";
				case eRoboticSystems::Panda: return "Panda";
				case eRoboticSystems::iiwa14: return "iiwa14";
				case eRoboticSystems::VISPA: return "VISPA";
				case eRoboticSystems::H1: return "H1";
				case eRoboticSystems::Cube: return "Cube";
				default: return "Unknown";
			}
		}

		inline std::string toString(eRoboticSystemFamilies family) {
			switch (family) {
				case eRoboticSystemFamilies::Unitree: return "Unitree Robotics";
				case eRoboticSystemFamilies::Universal: return "Universal Robots";
				case eRoboticSystemFamilies::Franka: return "Franka Robotics";
				case eRoboticSystemFamilies::KUKA: return "KUKA";
				case eRoboticSystemFamilies::Airbus: return "Airbus";
				case eRoboticSystemFamilies::Other: return "Other";
				default: return "Unknown";
			}
		}
	};

	inline std::unordered_map<eRoboticSystems, eRoboticSystemFamilies> getRobotSystemMap() { return RoboticSystems().robotMap; }
}