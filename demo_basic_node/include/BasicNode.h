#pragma once


#include "Node.h"
#include "FloatObject.h"

class BasicNode : public Node
{
private:
	FloatObject	input;
	FloatObject output;	
	bool recv_input;
protected:

	// Setup -- REQUIRED
	// Sets up Subscriptions and Publishing for the node. Registers member functions for execution.
	void Setup(int argc, char** argv);

	// SetNodeName -- REQUIRED
	// Used to specify name that identifies node in the network
	void SetNodeName(int argc, char** argv, std::string& nodeName);
private:
	// AppInit -- 
	// Example Initialization function.  Called after Setup().  Used for application specific initialization
	void AppInit();	

	// OnReceiveInput --
	// Example Input function. Called on notification of received data from subscription topic. Used to handle receiving new data.
	void OnReceiveInput();	

	// Process --
	// Example Core function. Called every iteration of control loop. Used to do any continuous process the node might require.
	void Process();

	// OnExit -- 
	// Example Exit function. Called right before control loop exits; before the application closes. Used to handle any clean up
	void OnExit();
};