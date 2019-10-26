#ifndef TESTSUB_H
#define TESTSUB_H
#include "Node.h"
#include "Line.h"
#include "Vehicle.h"
#include "LocationObject.h"
#include "WheelsMessage.h"

#include <vector>

class testSub : public Node
{
protected:
    void Setup(int argc, char** argv);
	void SetNodeName(int argc, char** argv, std::string& nodeName);
private:
    Vehicle _vehicle1, _vehicle2, _vehicle3, _vehicle4; 
    WheelsMessage * _veh1Wheels, * _veh2Wheels, * _veh3Wheels, * _veh4Wheels; 
    LocationObject _veh1Loc, _veh2Loc, _veh3Loc, _veh4Loc; 

    std::vector<Line> lines; 

    float update_interval, terminationTime; 

    void AppInit();
	void OnReceiveLocation();
	void OnReceiveLocation2();
    void OnReceiveLocation3();
   // void OnReceiveLocation4();
	bool Load(const char* filename);	
	void Process();
};

#endif