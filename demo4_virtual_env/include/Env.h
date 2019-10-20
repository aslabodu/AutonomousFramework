#pragma once

#include "Node.h"
#include "DetectedObject.h"
#include "LocationObject.h"
#include "FloatObject.h"
#include "WheelAction.h"
#include "Line.h"
#include "Vehicle.h"

#include <vector>

class Environment : public Node
{
private:		

	Vehicle vehicle1, vehicle2;
	WheelAction vehicle1_ticks, vehicle2_ticks;
	LocationObject vehicle1_location, vehicle2_location;
	DetectedObject vehicle1_detection, vehicle2_detection;

	std::vector<DetectedObject> objects;
	std::vector<Line> lines;

	LocationObject obstacle;

	float update_interval, terminationTime;

	//bool recv_location;

protected:
	void Setup(int argc, char** argv);
	void SetNodeName(int argc, char** argv, std::string& nodeName);
private:
	void AppInit();
	void OnReceiveVehicle1();
	void OnReceiveVehicle2();
	bool Load(const char* filename);	
	void Process();
	void DetectVehicle1();
	void DetectVehicle2();
};