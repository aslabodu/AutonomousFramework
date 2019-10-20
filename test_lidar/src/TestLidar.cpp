#include "TestLidar.h"

#include <cmath>
#include <cstdio>
#include <signal.h>
#include <unistd.h>


const char* parameterFile = "../catkin_ws/src/test_lidar/config/parameters.xml";

// ------------------------------------------
// IMPORTANT!! - Make sure there is a definition for CreateApplicationNode()
Node* CreateApplicationNode()
{
	return new TestLidar();        // Make sure to change this to correct Node class type
}
// ------------------------------------------


void termination_handler (int signum)
{
  Node::Get()->Terminate(); // Example call to terminate the application with OS control signal
}


void TestLidar::Setup(int argc, char** argv)
{
    // Example of subscribing to certain topic and connecting to object "input".    
	Subscribe("PHYSICAL_RANGE", &range);

	Publish("DETECTED_OBJECT", &object);	


	RegisterInputFunction("PHYSICAL_RANGE",static_cast<NodeFuncPtr>(&TestLidar::OnReceiveRange));

	RegisterInitFunction(static_cast<NodeFuncPtr>(&TestLidar::Init));
    // Example of registering "Process" to be a core function processed continuously
	RegisterCoreFunction(static_cast<NodeFuncPtr>(&TestLidar::Process));
}

void TestLidar::SetNodeName(int argc, char** argv, std::string& nodeName)
{
	nodeName = "TestLidar";
}

void TestLidar::Init()
{

	if(Load(parameterFile) == false)
    {
    	printf("Failed to Load File: %s\n", parameterFile);
    	//Node::Get()->Terminate();
    }
}

bool TestLidar::Load(const char* filename)
{
  char wd[256];
  getcwd(wd, 256);
  printf("Current Working Directory = %s\n", wd);

  tinyxml2::XMLDocument doc;
  tinyxml2::XMLError result = doc.LoadFile(filename);

  if(result != tinyxml2::XML_SUCCESS)
    return(false);

  tinyxml2::XMLElement* pRoot=doc.RootElement();

  for(tinyxml2::XMLElement* pElem=pRoot->FirstChildElement(); pElem; pElem=pElem->NextSiblingElement())
  {
    	std::string elementName = pElem->Value();

			if(elementName=="constants")
			{
				pElem->QueryFloatAttribute("avoidDistance",&avoidDistance);
			}
    }
  return(true);
}

void TestLidar::OnReceiveRange()
{
    printf("%f\n", range.GetValue());

    if(range.GetValue() > 0.0f && range.GetValue() <= avoidDistance)
    {
      printf("SENSED RANGE: %f\n", range.GetValue());
    	object.SetY(range.GetValue());
    	object.SetX(0);
    	object.SetFlagged(true);
    }
}

void TestLidar::Process()
{
 	// Example handle termination signal CTRL-C --- Call "termination_handler"
  	if (signal (SIGINT, termination_handler) == SIG_IGN)
    	signal (SIGINT, SIG_IGN);	

}





