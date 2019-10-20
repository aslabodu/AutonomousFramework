#pragma once


#include "Node.h"
#include "WheelAction.h"

#include <cmath>

class Encoder : public Node
{
private:
	WheelAction encoder_ticks, wheels;
protected:

	// Setup -- REQUIRED
	// Sets up Subscriptions and Publishing for the node. Registers member functions for execution.
	void Setup(int argc, char** argv);

	// SetNodeName -- REQUIRED
	// Used to specify name that identifies node in the network
	void SetNodeName(int argc, char** argv, std::string& nodeName);
private:

	void AppInit();	

	void OnReceiveInput();	

	void Process();

	void OnExit();

	void SaveDataToFile();

	// https://gmao.gsfc.nasa.gov/research/subseasonal/atlas/GEV-RV-html/GEV-RV-description.html
	inline float GEVinv(float k, float mu, float s, float p){
	    return (pow((-1)*(log(p)+1)/(k),-k)*s+mu);
	}

	inline float LogLogisticInv(float mu, float s, float p){
	    return (mu*pow(1/p-1,(-1/s)));
	}

	inline float LogisticInv(float mu, float s, float p){
	    return (-s*log(1/p-1)+mu);
	}
};