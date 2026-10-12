#include "snapshotFixtureIO.h"

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
