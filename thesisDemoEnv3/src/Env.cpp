#include "Env.h"
#include "tinyxml2.h"

#include "Clock.h"

#include <cmath>
#include <cstdio>
#include <signal.h>
#include <unistd.h>

const char* parameterFile = "../catkin_ws/src/thesisDemoEnv/config/parameters.xml";

// ------------------------------------------
// IMPORTANT!! - Make sure there is a definition for CreateApplicationNode()
Node* CreateApplicationNode()
{
	return new Environment();
}
// ------------------------------------------

void termination_handler (int signum)
{
  Node::Get()->Terminate();
}

void Environment::Setup(int argc, char** argv)
{
	//Publish(FindTopicName("ENV_VEH1_LOCATION_OUTPUT"), &_veh1Loc);

	//Publish(FindTopicName("ENV_VEH2_LOCATION_OUTPUT"), &_veh2Loc);

    Publish(FindTopicName("ENV_VEH3_LOCATION_OUTPUT"), &_veh3Loc);

//	Publish(FindTopicName("ENV_VEH4_LOCATION_OUTPUT"), &_veh4Loc);

	RegisterInitFunction(static_cast<NodeFuncPtr>(&Environment::AppInit));
	RegisterCoreFunction(static_cast<NodeFuncPtr>(&Environment::Process));
}

void Environment::SetNodeName(int argc, char** argv, std::string& nodeName)
{
	nodeName = "ENV3";
}

void Environment::AppInit()
{
	//std::cout << "init in environment " << std::endl; 
	sleep(3);

	srand(time(0));		// set random seed

	if(Load(parameterFile) == false)
    {
    	printf("Failed to Load File: %s\n", parameterFile);
    	Node::Get()->Terminate();
    }
; 
	//recv_location = false;
    ResetClock();
}

bool Environment::Load(const char* filename)
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

    	if(elementName=="time") // termination time initialization
		{
		  pElem->QueryFloatAttribute("t",&terminationTime);
		}

		if(elementName=="line")
		{
		  float a,b,c;
		  pElem->QueryFloatAttribute("a",&a);
		  pElem->QueryFloatAttribute("b",&b);
		  pElem->QueryFloatAttribute("c",&c);

		  float minX,maxX,minY,maxY;
		  pElem->QueryFloatAttribute("minX",&minX);
		  pElem->QueryFloatAttribute("maxX",&maxX);
		  pElem->QueryFloatAttribute("minY",&minY);
		  pElem->QueryFloatAttribute("maxY",&maxY);

		  Line line(a,b,c);
		  line.SetBounds(minX,maxX,minY,maxY);
		  lines.push_back(line);	  
		}

		// if(elementName=="environment")
		// {
		// 	pElem->QueryFloatAttribute("maxRange",&max_range);
		// 	pElem->QueryFloatAttribute("fieldOfView",&field_of_view);
		// 	pElem->QueryFloatAttribute("update",&update_interval);
		// }
/* 
		if(elementName=="vehicle1")
		{
			float x,y,theta;
		  pElem->QueryFloatAttribute("x",&x);
		  pElem->QueryFloatAttribute("y",&y);
		  pElem->QueryFloatAttribute("theta",&theta);

		  float halfWidth,leftWheelConstant,rightWheelConstant,field_of_view,max_range, wheelBase, wheelRadius;
		  pElem->QueryFloatAttribute("HW",&halfWidth);
		  pElem->QueryFloatAttribute("WB",&wheelBase);
		  pElem->QueryFloatAttribute("WR",&wheelRadius);
		  pElem->QueryFloatAttribute("LWC",&leftWheelConstant);
		  pElem->QueryFloatAttribute("RWC",&rightWheelConstant);
		  pElem->QueryFloatAttribute("FOV",&field_of_view);
		  pElem->QueryFloatAttribute("MR",&max_range);	

		  _vehicle1 = Vehicle(halfWidth, leftWheelConstant, rightWheelConstant, field_of_view, max_range, wheelRadius, wheelBase);
		  _vehicle1.SetLocation(x,y,theta);	
		}

		if(elementName=="vehicle2")
		{
			float x,y,theta;
		  pElem->QueryFloatAttribute("x",&x);
		  pElem->QueryFloatAttribute("y",&y);
		  pElem->QueryFloatAttribute("theta",&theta);


		  float halfWidth,leftWheelConstant,rightWheelConstant,field_of_view,max_range, wheelBase, wheelRadius;
		  pElem->QueryFloatAttribute("HW",&halfWidth);
		  pElem->QueryFloatAttribute("WB",&wheelBase);
		  pElem->QueryFloatAttribute("WR",&wheelRadius);
		  pElem->QueryFloatAttribute("LWC",&leftWheelConstant);
		  pElem->QueryFloatAttribute("RWC",&rightWheelConstant);
		  pElem->QueryFloatAttribute("FOV",&field_of_view);
		  pElem->QueryFloatAttribute("MR",&max_range);		

		  _vehicle2 = Vehicle(halfWidth, leftWheelConstant, rightWheelConstant, field_of_view, max_range, wheelRadius, wheelBase);
		  _vehicle2.SetLocation(x,y,theta);	
		}*/
 
        if(elementName=="vehicle3")
		{
		    float x,y,theta;
		  pElem->QueryFloatAttribute("x",&x);
		  pElem->QueryFloatAttribute("y",&y);
		  pElem->QueryFloatAttribute("theta",&theta);

		  float halfWidth,leftWheelConstant,rightWheelConstant,field_of_view,max_range, wheelBase, wheelRadius;
		  pElem->QueryFloatAttribute("HW",&halfWidth);
		  pElem->QueryFloatAttribute("WB",&wheelBase);
		  pElem->QueryFloatAttribute("WR",&wheelRadius);
		  pElem->QueryFloatAttribute("LWC",&leftWheelConstant);
		  pElem->QueryFloatAttribute("RWC",&rightWheelConstant);
		  pElem->QueryFloatAttribute("FOV",&field_of_view);
		  pElem->QueryFloatAttribute("MR",&max_range);	

		  _vehicle3 = Vehicle(halfWidth, leftWheelConstant, rightWheelConstant, field_of_view, max_range, wheelRadius, wheelBase);
		  _vehicle3.SetLocation(x,y,theta);	
		}
/* 
		if(elementName=="vehicle4")
		{
			float x,y,theta;
		  pElem->QueryFloatAttribute("x",&x);
		  pElem->QueryFloatAttribute("y",&y);
		  pElem->QueryFloatAttribute("theta",&theta);

		  float halfWidth,leftWheelConstant,rightWheelConstant,field_of_view,max_range, wheelBase, wheelRadius;
		  pElem->QueryFloatAttribute("HW",&halfWidth);
		  pElem->QueryFloatAttribute("WB",&wheelBase);
		  pElem->QueryFloatAttribute("WR",&wheelRadius);
		  pElem->QueryFloatAttribute("LWC",&leftWheelConstant);
		  pElem->QueryFloatAttribute("RWC",&rightWheelConstant);
		  pElem->QueryFloatAttribute("FOV",&field_of_view);
		  pElem->QueryFloatAttribute("MR",&max_range);		

		  _vehicle4 = Vehicle(halfWidth, leftWheelConstant, rightWheelConstant, field_of_view, max_range, wheelRadius, wheelBase);
		  _vehicle4.SetLocation(x,y,theta);	
		}*/
    }

  return(true);	
}

void Environment::Process()
{
    // ---- Termination Signal ------ //
  	if (signal (SIGINT, termination_handler) == SIG_IGN)
	  {
		  ROS_INFO("TERMINATION SIGNAL CALLED");
    	signal (SIGINT, SIG_IGN);
	  }

  ///  if (ElapsedTime() >= terminationTime) // termination when time elapsed
  //  	signal (SIGINT, SIG_IGN);
    if(CheckForMessage(0) == true)
    {
        float x,y,theta;

        if(GetNextMsgType(0) == 1)
        {   
			_veh3Wheels =  dynamic_cast<WheelsMessage *>(GetMessage(0)->Clone());
            _vehicle3.UpdateLocation(_veh3Wheels->GetLeftWheel(), _veh3Wheels->GetRightWheel());
            _vehicle3.GetLocation(x,y,theta);
            _veh3Loc.SetX(x);
            _veh3Loc.SetY(y);
            _veh3Loc.SetHeading(theta);
            _veh3Loc.SetFlagged(true);
		}
	}
}
