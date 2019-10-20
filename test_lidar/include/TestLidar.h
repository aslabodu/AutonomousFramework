#pragma once


#include "Node.h"
#include "FloatObject.h"
#include "DetectedObject.h"
#include "tinyxml2.h"


class TestLidar : public Node
{
private:
	FloatObject	range;

	DetectedObject object;

	float avoidDistance;

protected:

	// Setup -- REQUIRED
	// Sets up Subscriptions and Publishing for the node. Registers member functions for execution.
	void Setup(int argc, char** argv);

	// SetNodeName -- REQUIRED
	// Used to specify name that identifies node in the network
	void SetNodeName(int argc, char** argv, std::string& nodeName);
private:

	void OnReceiveRange();
	// Process --
	// Example Core function. Called every iteration of control loop. Used to do any continuous process the node might require.
	void Process();

	void Init();
	bool Load(const char* filename);


};