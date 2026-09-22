#include <global_planner/pathGainEvaluator.h>
#include <json/json.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace{

template<typename T>
T readValue(std::istream& stream){
	T value;
	stream.read(reinterpret_cast<char*>(&value),sizeof(T));
	if (!stream) throw std::runtime_error("truncated map binary");
	return value;
}

uint64_t fnv1a64(const std::string& path){
	std::ifstream stream(path,std::ios::binary);
	if (!stream) throw std::runtime_error("cannot open file for hash");
	uint64_t value = 14695981039346656037ull;
	char buffer[65536];
	while (stream){
		stream.read(buffer,sizeof(buffer));
		for (std::streamsize index=0; index<stream.gcount(); ++index){
			value ^= static_cast<unsigned char>(buffer[index]);
			value *= 1099511628211ull;
		}
	}
	return value;
}

std::string hex64(uint64_t value){
	std::ostringstream stream;
	stream << std::hex << std::setfill('0') << std::setw(16) << value;
	return stream.str();
}

Json::Value loadJson(const std::string& path){
	std::ifstream stream(path);
	if (!stream) throw std::runtime_error("cannot open json");
	Json::CharReaderBuilder builder;
	Json::Value value;
	std::string errors;
	if (!Json::parseFromStream(builder,stream,&value,&errors)){
		throw std::runtime_error("invalid json: "+errors);
	}
	return value;
}

mapManager::OccupancyMapSnapshot loadMap(const std::string& path){
	std::ifstream stream(path,std::ios::binary);
	if (!stream) throw std::runtime_error("cannot open map binary");
	char magic[8]; stream.read(magic,sizeof(magic));
	if (!stream || std::string(magic,7)!="CR1MAP1") throw std::runtime_error("invalid map magic");
	if (readValue<uint32_t>(stream)!=1) throw std::runtime_error("unsupported map schema");
	mapManager::OccupancyMapSnapshot snapshot;
	snapshot.version = readValue<uint64_t>(stream);
	snapshot.resolution = readValue<double>(stream);
	for (int axis=0; axis<3; ++axis) snapshot.mapMin(axis)=readValue<double>(stream);
	for (int axis=0; axis<3; ++axis) snapshot.mapMax(axis)=readValue<double>(stream);
	for (int axis=0; axis<3; ++axis) snapshot.dimensions(axis)=readValue<int32_t>(stream);
	snapshot.pMinLog=readValue<double>(stream);
	snapshot.pOccLog=readValue<double>(stream);
	const uint64_t count=readValue<uint64_t>(stream);
	if (count!=static_cast<uint64_t>(snapshot.dimensions(0))*snapshot.dimensions(1)*
		snapshot.dimensions(2)) throw std::runtime_error("map voxel count mismatch");
	snapshot.occupancy.resize(count);
	snapshot.inflated.resize(count);
	for (uint64_t index=0; index<count; ++index){
		const int8_t state=readValue<int8_t>(stream);
		const uint8_t inflated=readValue<uint8_t>(stream);
		if (state < -1 || state > 1 || inflated > 1) throw std::runtime_error("invalid map payload");
		snapshot.occupancy[index] = state == -1 ? snapshot.pMinLog-1.0 :
			(state == 1 ? snapshot.pOccLog : snapshot.pMinLog);
		snapshot.inflated[index] = inflated != 0;
	}
	if (stream.peek()!=std::char_traits<char>::eof()) throw std::runtime_error("trailing map data");
	return snapshot;
}

double number(const Json::Value& value){
	if (!value.isNumeric()) throw std::runtime_error("expected numeric json value");
	return value.asDouble();
}

}

int main(int argc,char** argv){
	try{
		if (argc!=3){
			std::cerr << "usage: path_gain_fixture_runner SNAPSHOT_DIR SPACING\n";
			return 2;
		}
		const std::string directory=argv[1];
		const double spacing=std::stod(argv[2]);
		const Json::Value manifest=loadJson(directory+"/manifest.json");
		if (manifest["schema"].asString()!="cerlab-r1-snapshot-v1" ||
			!manifest["complete"].asBool()) throw std::runtime_error("invalid manifest");
		const std::string mapPath=directory+"/"+manifest["map_file"].asString();
		const std::string plannerPath=directory+"/"+manifest["planner_file"].asString();
		if (hex64(fnv1a64(mapPath))!=manifest["map_fnv1a64"].asString() ||
			hex64(fnv1a64(plannerPath))!=manifest["planner_fnv1a64"].asString()){
			throw std::runtime_error("snapshot hash mismatch");
		}
		const Json::Value planner=loadJson(plannerPath);
		const mapManager::OccupancyMapSnapshot snapshot=loadMap(mapPath);
		if (planner["map_version"].asUInt64()!=snapshot.version ||
			manifest["map_version"].asUInt64()!=snapshot.version){
			throw std::runtime_error("snapshot version mismatch");
		}
		globalPlanner::PathGainVisibilityConfig config;
		config.horizontalFov=number(planner["sensor_model"]["horizontal_fov"]);
		config.verticalFov=number(planner["sensor_model"]["vertical_fov"]);
		config.dmax=number(planner["sensor_model"]["dmax"]);
		for (int axis=0; axis<3; ++axis){
			config.planningMin(axis)=number(planner["planning_region"]["min"][axis]);
			config.planningMax(axis)=number(planner["planning_region"]["max"][axis]);
		}
		globalPlanner::PathGainEvaluator evaluator(snapshot,config);
		Json::Value output;
		output["schema"]="cerlab-i1-cpp-path-gain-v1";
		output["map_version"]=Json::UInt64(snapshot.version);
		output["spacing"]=spacing;
		for (const Json::Value& candidate : planner["candidate_paths"]){
			std::vector<globalPlanner::PathGainWaypoint> waypoints;
			for (const Json::Value& waypoint : candidate["waypoints"]){
				globalPlanner::PathGainWaypoint value;
				for (int axis=0; axis<3; ++axis) value.position(axis)=number(waypoint["position"][axis]);
				value.yaw=number(waypoint["yaw"]);
				waypoints.push_back(value);
			}
			const double estimatedTime=candidate["legacy_metrics"].get("estimated_time",0.0).asDouble();
			const auto result=evaluator.evaluate(waypoints,spacing,estimatedTime,snapshot.version);
			Json::Value row;
			row["candidate_id"]=candidate["id"];
			row["valid"]=result.valid;
			row["sample_count"]=result.sampleCount;
			row["raw_gain"]=Json::UInt64(result.rawGain);
			row["unique_gain"]=Json::UInt64(result.uniqueGain);
			row["duplicate_ratio"]=result.duplicateRatio;
			row["unique_utility"]=result.uniqueUtility;
			row["marginal_gains"]=Json::Value(Json::arrayValue);
			row["unique_addresses"]=Json::Value(Json::arrayValue);
			for (uint64_t marginal : result.marginalGains)
				row["marginal_gains"].append(Json::UInt64(marginal));
			for (uint32_t address : result.uniqueAddresses)
				row["unique_addresses"].append(Json::UInt(address));
			output["candidates"].append(row);
		}
		Json::StreamWriterBuilder builder;
		builder["indentation"]="";
		std::cout << Json::writeString(builder,output) << "\n";
		return 0;
	}
	catch (const std::exception& error){
		std::cerr << error.what() << "\n";
		return 1;
	}
}
