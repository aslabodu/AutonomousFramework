#pragma once


#include "Node.h"
#include "DetectedObject.h"
#include "FloatObject.h"
#include "LocationObject.h"
#include "WheelAction.h"

#include <map>

class Planner : public Node
{
private:

	DetectedObject detected_object;
	WheelAction wheels;

	bool recv_object;
	float init_timer;
	float steering_timer;
	float detection_timer;
	bool halt;

	bool Forward;
	bool Pause;
	bool Turn;
	bool Init;

protected:

	void Setup(int argc, char** argv);
	void SetNodeName(int argc, char** argv, std::string& nodeName);
private:

	void AppInit();
	void OnReceiveObject();
	void Process();
	void OnExit();
};