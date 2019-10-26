#include "testSub.h"
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
	return new testSub();
}
// ------------------------------------------

void termination_handler (int signum)
{
  Node::Get()->Terminate();
}

void testSub::Setup(int argc, char** argv)
{
    std::string input1, input2, input3;

    input2 = FindTopicName("input2");
	Subscribe(input2, &_veh2Loc);		
	RegisterInputFunction(input2,static_cast<NodeFuncPtr>(&testSub::OnReceiveLocation2));

	input1 = FindTopicName("input1");
	Subscribe(input1, &_veh1Loc);		
	RegisterInputFunction(input1,static_cast<NodeFuncPtr>(&testSub::OnReceiveLocation));

	input3 = FindTopicName("input3");
	Subscribe(input3, &_veh3Loc);		
	RegisterInputFunction(input3,static_cast<NodeFuncPtr>(&testSub::OnReceiveLocation3));

    /* 
	Publish(FindTopicName("ENV_VEH1_LOCATION_OUTPUT"), &_veh1Loc);

	Publish(FindTopicName("ENV_VEH2_LOCATION_OUTPUT"), &_veh2Loc);

    Publish(FindTopicName("ENV_VEH3_LOCATION_OUTPUT"), &_veh3Loc);

	Publish(FindTopicName("ENV_VEH4_LOCATION_OUTPUT"), &_veh4Loc); */

	RegisterInitFunction(static_cast<NodeFuncPtr>(&testSub::AppInit));
	RegisterCoreFunction(static_cast<NodeFuncPtr>(&testSub::Process));
}

void testSub::SetNodeName(int argc, char** argv, std::string& nodeName)
{
	nodeName = "TESTSUB";
}

void testSub::AppInit()
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

bool testSub::Load(const char* filename)
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
		}

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
		}
    }

  return(true);	
}

void testSub::Process()
{
    // ---- Termination Signal ------ //
  	if (signal (SIGINT, termination_handler) == SIG_IGN)
	  {
		  ROS_INFO("TERMINATION SIGNAL CALLED");
    	signal (SIGINT, SIG_IGN);
	  }

}

void testSub::OnReceiveLocation()
{
	printf("Received Location:  X:%f| Y:%f| Theta:%f \n", _veh1Loc.GetX(), _veh1Loc.GetY(), _veh1Loc.GetHeading());
	
//worker.drawRover(veh1Loc.GetX(), veh1Loc.GetY(), veh1Loc.GetHeading(),1);
}

void testSub::OnReceiveLocation2(){

	printf("Received Location 2:  X:%f| Y:%f| Theta:%f \n", _veh2Loc.GetX(), _veh2Loc.GetY(), _veh2Loc.GetHeading());
	
//	worker.drawRover(veh2Loc.GetX(), veh2Loc.GetY(), veh2Loc.GetHeading(),2);	
}

void testSub::OnReceiveLocation3()
{
	printf("Received Location 3:  X:%f| Y:%f| Theta:%f \n", _veh3Loc.GetX(), _veh3Loc.GetY(), _veh3Loc.GetHeading());
	
//	worker.drawRover(veh3Loc.GetX(), veh3Loc.GetY(), veh3Loc.GetHeading(),3);
}
