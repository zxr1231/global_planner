#include <global_planner/dep.h>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>

namespace {
	bool makeDirectories(const std::string& path){
		if (path.empty()) return false;
		std::string current;
		for (size_t i = 0; i < path.size(); ++i){
			current.push_back(path[i]);
			if (path[i] != '/' || current.size() == 1) continue;
			if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) return false;
		}
		return ::mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
	}

	uint64_t fnv1a64(const std::string& path){
		std::ifstream input(path.c_str(), std::ios::binary);
		uint64_t hash = 14695981039346656037ULL;
		char buffer[65536];
		while (input.good()){
			input.read(buffer, sizeof(buffer));
			for (std::streamsize i = 0; i < input.gcount(); ++i){
				hash ^= static_cast<unsigned char>(buffer[i]);
				hash *= 1099511628211ULL;
			}
		}
		return hash;
	}

	std::string hex64(uint64_t value){
		std::ostringstream stream;
		stream << std::hex << std::setw(16) << std::setfill('0') << value;
		return stream.str();
	}

	template <typename T>
	void writeBinary(std::ofstream& output, const T& value){
		output.write(reinterpret_cast<const char*>(&value), sizeof(T));
	}

	bool samePath(const std::vector<std::shared_ptr<PRM::Node>>& lhs,
				  const std::vector<std::shared_ptr<PRM::Node>>& rhs){
		if (lhs.size() != rhs.size()) return false;
		for (size_t i = 0; i < lhs.size(); ++i){
			if (lhs[i] != rhs[i]) return false;
		}
		return true;
	}
}

namespace globalPlanner {
	bool DEP::exportDiagnosticSnapshot(){
		return this->exportDiagnosticSnapshotNamed("snapshot", this->planningSequence_, "global_plan");
	}

	bool DEP::exportExecutionDiagnosticSnapshot(uint64_t executionSequence){
		if (!this->diagnosticSnapshotEnabled_) return false;
		return this->exportDiagnosticSnapshotNamed("execution", executionSequence, "execution_start");
	}

	bool DEP::exportDiagnosticSnapshotNamed(const std::string& prefix, uint64_t sequence,
										 const std::string& captureKind){
		if (!this->map_ || this->diagnosticSnapshotDir_.empty()) return false;
		if (!makeDirectories(this->diagnosticSnapshotDir_)){
			ROS_ERROR("[DEP][R1] Cannot create snapshot root: %s", this->diagnosticSnapshotDir_.c_str());
			return false;
		}

		std::ostringstream name;
		name << prefix << "_" << std::setw(6) << std::setfill('0') << sequence;
		const std::string finalDir = this->diagnosticSnapshotDir_ + "/" + name.str();
		const std::string tempDir = finalDir + ".tmp";
		struct stat info;
		if (::stat(finalDir.c_str(), &info) == 0 || ::stat(tempDir.c_str(), &info) == 0){
			ROS_WARN("[DEP][R1] Snapshot path already exists; refusing overwrite: %s", finalDir.c_str());
			return false;
		}
		if (::mkdir(tempDir.c_str(), 0755) != 0){
			ROS_ERROR("[DEP][R1] Cannot create temporary snapshot directory: %s", tempDir.c_str());
			return false;
		}

		const mapManager::OccupancyMapSnapshot mapSnapshot = this->map_->captureSnapshot();
		const std::string mapPath = tempDir + "/map.bin";
		std::ofstream mapOutput(mapPath.c_str(), std::ios::binary | std::ios::trunc);
		const char magic[8] = {'C','R','1','M','A','P','1','\0'};
		mapOutput.write(magic, sizeof(magic));
		const uint32_t schema = 1;
		writeBinary(mapOutput, schema);
		writeBinary(mapOutput, mapSnapshot.version);
		writeBinary(mapOutput, mapSnapshot.resolution);
		for (int i = 0; i < 3; ++i) writeBinary(mapOutput, mapSnapshot.mapMin(i));
		for (int i = 0; i < 3; ++i) writeBinary(mapOutput, mapSnapshot.mapMax(i));
		for (int i = 0; i < 3; ++i){
			const int32_t dimension = mapSnapshot.dimensions(i);
			writeBinary(mapOutput, dimension);
		}
		writeBinary(mapOutput, mapSnapshot.pMinLog);
		writeBinary(mapOutput, mapSnapshot.pOccLog);
		const uint64_t voxelCount = mapSnapshot.occupancy.size();
		writeBinary(mapOutput, voxelCount);
		for (size_t i = 0; i < mapSnapshot.occupancy.size(); ++i){
			const int8_t occupancyState = mapSnapshot.occupancy[i] < mapSnapshot.pMinLog ? -1 :
				(mapSnapshot.occupancy[i] >= mapSnapshot.pOccLog ? 1 : 0);
			const uint8_t inflated = mapSnapshot.inflated[i] ? 1 : 0;
			writeBinary(mapOutput, occupancyState);
			writeBinary(mapOutput, inflated);
		}
		mapOutput.close();
		if (!mapOutput){
			ROS_ERROR("[DEP][R1] Failed writing map snapshot: %s", mapPath.c_str());
			return false;
		}

		std::vector<std::shared_ptr<PRM::Node>> nodes(this->prmNodeVec_.begin(), this->prmNodeVec_.end());
		std::sort(nodes.begin(), nodes.end(), [](const std::shared_ptr<PRM::Node>& a,
											 const std::shared_ptr<PRM::Node>& b){
			for (int axis = 0; axis < 3; ++axis){
				if (a->pos(axis) < b->pos(axis)) return true;
				if (a->pos(axis) > b->pos(axis)) return false;
			}
			return a.get() < b.get();
		});
		std::unordered_map<const PRM::Node*, size_t> nodeIds;
		for (size_t i = 0; i < nodes.size(); ++i) nodeIds[nodes[i].get()] = i;

		std::vector<std::pair<size_t, size_t>> edges;
		for (size_t i = 0; i < nodes.size(); ++i){
			for (const auto& adjacent : nodes[i]->adjNodes){
				auto found = nodeIds.find(adjacent.get());
				if (found != nodeIds.end() && i < found->second) edges.emplace_back(i, found->second);
			}
		}
		std::sort(edges.begin(), edges.end());

		int selectedCandidate = -1;
		for (size_t i = 0; i < this->candidatePaths_.size(); ++i){
			if (samePath(this->candidatePaths_[i], this->bestPath_)) selectedCandidate = static_cast<int>(i);
		}

		const std::string graphPath = tempDir + "/planner.json";
		std::ofstream graph(graphPath.c_str(), std::ios::trunc);
		graph << std::setprecision(17);
		graph << "{\n  \"schema\": \"cerlab-r1-planner-v1\",\n";
		graph << "  \"planning_sequence\": " << sequence << ",\n";
		graph << "  \"capture_kind\": \"" << captureKind << "\",\n";
		graph << "  \"global_planning_sequence\": " << this->planningSequence_ << ",\n";
		graph << "  \"random_seed\": " << this->randomSeed_ << ",\n";
		graph << "  \"map_version\": " << mapSnapshot.version << ",\n";
		graph << "  \"vehicle\": {\"position\": [" << this->position_(0) << ", " << this->position_(1)
			<< ", " << this->position_(2) << "], \"yaw\": " << this->currYaw_ << "},\n";
		graph << "  \"sensor_model\": {\"horizontal_fov\": " << this->horizontalFOV_
			<< ", \"vertical_fov\": " << this->verticalFOV_ << ", \"dmin\": " << this->dmin_
			<< ", \"dmax\": " << this->dmax_ << ", \"yaw_samples\": " << this->yaws_.size()
			<< ", \"visibility_model\": \"legacy_inflated_occupied_line\"},\n";
		graph << "  \"path_gain_contract\": {\"schema_version\": " << this->pathGainSchemaVersion_
			<< ", \"configured_mode\": \"" << pathGainModeName(this->pathGainMode_)
			<< "\", \"selection_mode\": \"legacy\", \"sample_spacing\": "
			<< this->pathGainSampleSpacing_
			<< ", \"unique_evaluator_available\": "
			<< (this->uniqueGainEvaluatorAvailable_ ? "true" : "false")
			<< ", \"completion_gain_mode\": \"legacy\"},\n";
		graph << "  \"planning_region\": {\"min\": [" << this->globalRegionMin_(0) << ", "
			<< this->globalRegionMin_(1) << ", " << this->globalRegionMin_(2) << "], \"max\": ["
			<< this->globalRegionMax_(0) << ", " << this->globalRegionMax_(1) << ", "
			<< this->globalRegionMax_(2) << "]},\n";
		graph << "  \"motion_model\": {\"velocity\": " << this->vel_ << ", \"angular_velocity\": "
			<< this->angularVel_ << ", \"yaw_penalty_weight\": " << this->yawPenaltyWeight_ << "},\n";
		graph << "  \"roadmap_nodes\": [\n";
		for (size_t i = 0; i < nodes.size(); ++i){
			std::vector<std::pair<double, int>> yawGains(nodes[i]->yawNumVoxels.begin(), nodes[i]->yawNumVoxels.end());
			std::sort(yawGains.begin(), yawGains.end());
			graph << "    {\"id\": " << i << ", \"position\": [" << nodes[i]->pos(0) << ", "
				<< nodes[i]->pos(1) << ", " << nodes[i]->pos(2) << "], \"legacy_node_gain\": "
				<< nodes[i]->numVoxels << ", \"legacy_yaw_gains\": [";
			for (size_t y = 0; y < yawGains.size(); ++y){
				if (y) graph << ", ";
				graph << "[" << yawGains[y].first << ", " << yawGains[y].second << "]";
			}
			graph << "]}" << (i + 1 == nodes.size() ? "\n" : ",\n");
		}
		graph << "  ],\n  \"roadmap_edges\": [";
		for (size_t i = 0; i < edges.size(); ++i){
			if (i) graph << ", ";
			graph << "[" << edges[i].first << ", " << edges[i].second << "]";
		}
		graph << "],\n  \"goal_candidates\": [";
		for (size_t i = 0; i < this->goalCandidates_.size(); ++i){
			if (i) graph << ", ";
			const auto& goal = this->goalCandidates_[i];
			graph << "{\"position\": [" << goal->pos(0) << ", " << goal->pos(1) << ", " << goal->pos(2)
				<< "], \"legacy_node_gain\": " << goal->numVoxels << "}";
		}
		graph << "],\n  \"candidate_paths\": [\n";
		for (size_t p = 0; p < this->candidatePaths_.size(); ++p){
			graph << "    {\"id\": " << p;
			if (p < this->candidateLegacyMetrics_.size()){
				const auto& metrics = this->candidateLegacyMetrics_[p];
				graph << ", \"legacy_metrics\": {\"valid\": " << (metrics.valid ? "true" : "false")
					<< ", \"gain\": " << metrics.gain
					<< ", \"path_length\": " << metrics.pathLength
					<< ", \"yaw_distance\": " << metrics.yawDistance
					<< ", \"estimated_time\": " << metrics.estimatedTime
					<< ", \"score\": " << metrics.score << "}";
			}
			graph << ", \"raw_waypoints\": [";
			if (p < this->candidateRawPaths_.size()){
				const auto& rawPath = this->candidateRawPaths_[p];
				for (size_t i = 0; i < rawPath.size(); ++i){
					if (i) graph << ", ";
					double yaw = rawPath[i]->getBestYaw();
					if (i + 1 < rawPath.size()){
						const Eigen::Vector3d diff = rawPath[i + 1]->pos - rawPath[i]->pos;
						yaw = std::atan2(diff(1), diff(0));
					}
					graph << "{\"position\": [" << rawPath[i]->pos(0) << ", "
						<< rawPath[i]->pos(1) << ", " << rawPath[i]->pos(2)
						<< "], \"yaw\": " << yaw << "}";
				}
			}
			graph << "], \"waypoints\": [";
			const auto& path = this->candidatePaths_[p];
			for (size_t i = 0; i < path.size(); ++i){
				if (i) graph << ", ";
				double yaw = path[i]->getBestYaw();
				if (i + 1 < path.size()){
					const Eigen::Vector3d diff = path[i + 1]->pos - path[i]->pos;
					yaw = std::atan2(diff(1), diff(0));
				}
				graph << "{\"position\": [" << path[i]->pos(0) << ", " << path[i]->pos(1) << ", "
					<< path[i]->pos(2) << "], \"yaw\": " << yaw << "}";
			}
			graph << "]}" << (p + 1 == this->candidatePaths_.size() ? "\n" : ",\n");
		}
		graph << "  ],\n  \"selected_candidate\": " << selectedCandidate << ",\n";
		graph << "  \"legacy_best_path_gain\": " << this->bestPathGain_ << "\n}\n";
		graph.close();
		if (!graph){
			ROS_ERROR("[DEP][R1] Failed writing planner snapshot: %s", graphPath.c_str());
			return false;
		}

		const std::string manifestPath = tempDir + "/manifest.json";
		std::ofstream manifest(manifestPath.c_str(), std::ios::trunc);
		manifest << "{\n  \"schema\": \"cerlab-r1-snapshot-v1\",\n"
			<< "  \"complete\": true,\n"
			<< "  \"planning_sequence\": " << sequence << ",\n"
			<< "  \"capture_kind\": \"" << captureKind << "\",\n"
			<< "  \"map_version\": " << mapSnapshot.version << ",\n"
			<< "  \"map_file\": \"map.bin\",\n"
			<< "  \"map_fnv1a64\": \"" << hex64(fnv1a64(mapPath)) << "\",\n"
			<< "  \"planner_file\": \"planner.json\",\n"
			<< "  \"planner_fnv1a64\": \"" << hex64(fnv1a64(graphPath)) << "\"\n}\n";
		manifest.close();
		if (!manifest || std::rename(tempDir.c_str(), finalDir.c_str()) != 0){
			ROS_ERROR("[DEP][R1] Failed committing snapshot: %s", finalDir.c_str());
			return false;
		}
		ROS_INFO("[DEP][R1] Wrote immutable diagnostic snapshot: %s", finalDir.c_str());
		return true;
	}
}
