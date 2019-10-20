#include "location_combiner.h"
// ------------------------------------------
// IMPORTANT!! - Make sure there is a definition for CreateApplicationNode()
Node* CreateApplicationNode()
{
    return new location_combiner();
}
// ------------------------------------------


// Set Node Name
void location_combiner::SetNodeName(int argc, char** argv, std::string& nodeName)
{
    nodeName = "location_combiner";
}

// Create Objects
void location_combiner::CreateObjects(int argc, char** argv, SerialObject*& physicalLocation, SerialObject*& virtualLocation, SerialObject*& outputLocation)
{
	physicalLocation = new LocationObject();
	virtualLocation = new LocationObject();
	outputLocation = new LocationObject();
}

// Set Topic Names
void location_combiner::SetTopicNames(int argc, char** argv, std::string& physicalName, std::string& virtualName, std::string& outputName)
{
	physicalName = FindTopicName("input1");
	virtualName = FindTopicName("input2");
    outputName = FindTopicName("output1");
}

// Set Mode
// Heading combiner uses ROS parameter server to set mode of operation
// Default is mode 0 (physical)
void location_combiner::SetMode(int argc, char**argv, int& mode)
{
    // Obtain mode from ROS parameter server
    if(ros::param::has("~mode"))
        ros::param::get("~mode", mode);
    else
        mode = 0;
}


void location_combiner::Combine(SerialObject* physicalLocation, SerialObject* virtualLocation, SerialObject* outputLocation)
{   
	LocationObject* physicalLoc = static_cast<LocationObject*>(physicalLocation);
	LocationObject* virtualLoc = static_cast<LocationObject*>(virtualLocation);
	LocationObject* outputLoc = static_cast<LocationObject*>(outputLocation);


	if(GetMode() == 0)
	{
		*(outputLoc)=*(physicalLoc);	// copy physical
	}
	else if(GetMode() == 1)
	{
		*(outputLoc)=*(virtualLoc);	// copy virtual
	}
	else if(GetMode() == 2)
	{
		// // Compare distance between object positions
		// if(physicalLoc->GetRange() < virtualLoc->GetRange())
		// 	*(outputLoc)=*(physicalLoc);	// copy physical
		// 	*(outputLoc)=*(virtualLoc);	// copy virtual
	}
}

