#ifndef ENV_H
#define ENV_H

#include "Node.h"
#include "Line.h"
#include "Vehicle.h"
#include "LocationObject.h"
#include "WheelsMessage.h"

#include <vector>

class Environment : public Node
{
protected:
    void Setup(int argc, char** argv);
	void SetNodeName(int argc, char** argv, std::string& nodeName);
private:
    Vehicle _vehicle3; //, _vehicle2, _vehicle3, _vehicle4; 
    WheelsMessage * _veh3Wheels; //, * _veh2Wheels, * _veh3Wheels, * _veh4Wheels; 
    LocationObject _veh3Loc; //, _veh2Loc, _veh3Loc, _veh4Loc; 

    std::vector<Line> lines; 

    float update_interval, terminationTime; 

    void AppInit();
	void OnReceiveVehicle1();
	void OnReceiveVehicle2();
    void OnReceiveVehicle3();
    void OnReceiveVehicle4();
	bool Load(const char* filename);	
	void Process();
};

#endif