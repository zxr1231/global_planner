#ifndef GLOBAL_PLANNER_PATH_GAIN_CONTRACT_H
#define GLOBAL_PLANNER_PATH_GAIN_CONTRACT_H

#include <cmath>
#include <stdexcept>
#include <string>

namespace globalPlanner{

constexpr int PATH_GAIN_SCHEMA_VERSION = 1;

enum class PathGainMode{
	LEGACY,
	UNIQUE_SHADOW,
	UNIQUE_ONLINE
};

inline const char* pathGainModeName(PathGainMode mode){
	switch (mode){
		case PathGainMode::LEGACY: return "legacy";
		case PathGainMode::UNIQUE_SHADOW: return "unique_shadow";
		case PathGainMode::UNIQUE_ONLINE: return "unique_online";
	}
	throw std::logic_error("unhandled path gain mode");
}

inline PathGainMode parsePathGainMode(const std::string& value){
	if (value == "legacy") return PathGainMode::LEGACY;
	if (value == "unique_shadow") return PathGainMode::UNIQUE_SHADOW;
	if (value == "unique_online") return PathGainMode::UNIQUE_ONLINE;
	throw std::invalid_argument("path gain mode must be legacy, unique_shadow, or unique_online");
}

inline bool requestsUniqueEvaluation(PathGainMode mode){
	return mode != PathGainMode::LEGACY;
}

inline bool requestsUniqueSelection(PathGainMode mode){
	return mode == PathGainMode::UNIQUE_ONLINE;
}

inline void validatePathGainContract(int schemaVersion, PathGainMode mode,
									 double sampleSpacing, bool uniqueEvaluatorAvailable,
									 bool uniqueOnlineAvailable = false){
	if (schemaVersion != PATH_GAIN_SCHEMA_VERSION){
		throw std::invalid_argument("unsupported path gain schema version");
	}
	if (!std::isfinite(sampleSpacing) || sampleSpacing <= 0.0){
		throw std::invalid_argument("path gain sample spacing must be finite and positive");
	}
	if (requestsUniqueEvaluation(mode) && !uniqueEvaluatorAvailable){
		throw std::logic_error("requested unique path gain mode is declared but not implemented");
	}
	if (requestsUniqueSelection(mode) && !uniqueOnlineAvailable){
		throw std::logic_error("unique online selection is not enabled at this phase");
	}
}

} // namespace globalPlanner

#endif
