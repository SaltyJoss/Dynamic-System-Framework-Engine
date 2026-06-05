// DSFE_GUI SystemMap.h
#pragma once

#include <unordered_map>
#include <string>

namespace platform {
	enum class eRoboticArms {
		Z1 = 0,
		UR5e = 1,
		Panda = 2,
		iiwa14 = 3,
		VISPA = 4,
		H1 = 5
	};

	enum class eRoboticArmFamilies {
		Unitree = 0,
		Universal = 1,
		Franka = 2,
		KUKA = 3,
		Airbus = 4
	};

	struct RoboticArms {
		std::unordered_map<eRoboticArms, eRoboticArmFamilies> armMap = {
			{ eRoboticArms::Z1, eRoboticArmFamilies::Unitree },
			{ eRoboticArms::UR5e, eRoboticArmFamilies::Universal },
			{ eRoboticArms::Panda, eRoboticArmFamilies::Franka },
			{ eRoboticArms::iiwa14, eRoboticArmFamilies::KUKA },
			{ eRoboticArms::VISPA, eRoboticArmFamilies::Airbus },
			{ eRoboticArms::H1, eRoboticArmFamilies::Unitree }
		};

		inline std::string toString(eRoboticArms arm) {
			switch (arm) {
				case eRoboticArms::Z1: return "Z1";
				case eRoboticArms::UR5e: return "UR5e";
				case eRoboticArms::Panda: return "Panda";
				case eRoboticArms::iiwa14: return "iiwa14";
				case eRoboticArms::VISPA: return "VISPA";
				case eRoboticArms::H1: return "H1";
				default: return "Unknown";
			}
		}

		inline std::string toString(eRoboticArmFamilies family) {
			switch (family) {
				case eRoboticArmFamilies::Unitree: return "Unitree Robotics";
				case eRoboticArmFamilies::Universal: return "Universal Robots";
				case eRoboticArmFamilies::Franka: return "Franka Robotics";
				case eRoboticArmFamilies::KUKA: return "KUKA";
				case eRoboticArmFamilies::Airbus: return "Airbus";
				default: return "Unknown";
			}
		}
	};

	std::unordered_map<eRoboticArms, eRoboticArmFamilies> getRoboticArmMap() { return RoboticArms().armMap; }
}