// DSFE_GUI SystemMap.h
#pragma once

#include <unordered_map>
#include <string>

namespace platform {
	/*
	 * Robotic Rigid Body Systems and Families
	 */
	// Free Body Systems
	enum class eRoboticSystems {
		Z1 = 0,
		UR5e = 1,
		Panda = 2,
		iiwa14 = 3,
		VISPA = 4,
		H1 = 5,
	};
	// Robotic System Families
	enum class eRoboticSystemFamilies {
		Unitree = 0,
		Universal = 1,
		Franka = 2,
		KUKA = 3,
		Airbus = 4,
		Other = 5
	};
	// Robotic Systems Mapping
	struct RoboticSystems {
		// Mapping of robotic systems to their families
		std::unordered_map<eRoboticSystems, eRoboticSystemFamilies> robotMap = {
			{ eRoboticSystems::Z1, eRoboticSystemFamilies::Unitree },
			{ eRoboticSystems::UR5e, eRoboticSystemFamilies::Universal },
			{ eRoboticSystems::Panda, eRoboticSystemFamilies::Franka },
			{ eRoboticSystems::iiwa14, eRoboticSystemFamilies::KUKA },
			{ eRoboticSystems::VISPA, eRoboticSystemFamilies::Airbus },
			{ eRoboticSystems::H1, eRoboticSystemFamilies::Unitree },
		};
		// Helper function to convert eRoboticSystems enum to string
		inline std::string toString(eRoboticSystems sys) {
			switch (sys) {
				case eRoboticSystems::Z1: return "Z1";
				case eRoboticSystems::UR5e: return "UR5e";
				case eRoboticSystems::Panda: return "Panda";
				case eRoboticSystems::iiwa14: return "iiwa14";
				case eRoboticSystems::VISPA: return "VISPA";
				case eRoboticSystems::H1: return "H1";
				default: return "Unknown";
			}
		}
		// Helper function to convert eRoboticSystemFamilies enum to string
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

	/*
	 * Free Rigid Body Systems and Families
	 */
	// Free Body Systems
	enum class eFreeBodies {
		cube = 0
	};
	// Free Body Families
	enum class eFreeBodyFamilies {
		shapes = 0,
		other = 1
	};
	// Free Body Systems Mapping
	struct FreeBodies {
		// Mapping of free bodies to their families
		std::unordered_map<eFreeBodies, eFreeBodyFamilies> bodyMap = {
			{ eFreeBodies::cube, eFreeBodyFamilies::shapes }
		};
		// Helper function to convert eFreeBodies enum to string
		inline std::string toString(eFreeBodies sys) {
			switch (sys) {
				case eFreeBodies::cube: return "Cube";
				default: return "Unknown";
			}
		}
		// Helper function to convert eFreeBodyFamilies enum to string
		inline std::string toString(eFreeBodyFamilies family) {
			switch (family) {
				case eFreeBodyFamilies::shapes: return "Shapes";
				case eFreeBodyFamilies::other: return "Other";
				default: return "Unknown";
			}
		}
	};

	/*
	 * Accessors for system mappings
	 */
	// Returns an unordered_map of robotic systems to their families
	inline std::unordered_map<eRoboticSystems, eRoboticSystemFamilies> getRobotSystemMap() { return RoboticSystems().robotMap; }
	// Returns an unordered_map of free bodies to their families
	inline std::unordered_map<eFreeBodies, eFreeBodyFamilies> getFreeBodySystemMap() { return FreeBodies().bodyMap; }
}