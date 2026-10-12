#pragma once
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

