#pragma once


#include "Node.h"
#include "WheelAction.h"

class BasicNode : public Node
{
private:
	WheelAction encoder_ticks;
	std::vector<int> leftWheelData;
	std::vector<int> rightWheelData;
	bool enabled;
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
};