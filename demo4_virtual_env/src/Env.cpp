#include "Env.h"
#include "tinyxml2.h"

#include "Clock.h"

#include <cmath>
#include <cstdio>
#include <signal.h>
#include <unistd.h>

const char* parameterFile = "../catkin_ws/src/demo4_virtual_env/config/parameters.xml";

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

	std::string input1 = FindTopicName("ENV_VEH1_INPUT");
	Subscribe(input1, &vehicle1_ticks);	
	RegisterInputFunction(input1,static_cast<NodeFuncPtr>(&Environment::OnReceiveVehicle1));

	std::string input2 = FindTopicName("ENV_VEH2_INPUT");
	Subscribe(input2, &vehicle2_ticks);	
	RegisterInputFunction(input2,static_cast<NodeFuncPtr>(&Environment::OnReceiveVehicle2));

	Publish(FindTopicName("ENV_VEH1_DETECTION_OUTPUT"), &vehicle1_detection);

	Publish(FindTopicName("ENV_VEH2_DETECTION_OUTPUT"), &vehicle2_detection);

	Publish(FindTopicName("ENV_OBSTACLE_OUTPUT"), &obstacle);	

	Publish(FindTopicName("ENV_VEH1_LOCATION_OUTPUT"), &vehicle1_location);

	Publish(FindTopicName("ENV_VEH2_LOCATION_OUTPUT"), &vehicle2_location);

	RegisterInitFunction(static_cast<NodeFuncPtr>(&Environment::AppInit));
	RegisterCoreFunction(static_cast<NodeFuncPtr>(&Environment::Process));
}

void Environment::SetNodeName(int argc, char** argv, std::string& nodeName)
{
	nodeName = "Environment";
}

void Environment::AppInit()
{
	sleep(3);

	srand(time(0));		// set random seed

	if(Load(parameterFile) == false)
    {
    	printf("Failed to Load File: %s\n", parameterFile);
    	Node::Get()->Terminate();
    }


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

    	if(elementName=="object")
		{
		  float x,y;
		  pElem->QueryFloatAttribute("x",&x);
		  pElem->QueryFloatAttribute("y",&y);		  

		  DetectedObject obj;
		  obj.SetX(x);
		  obj.SetY(y);		  
		  objects.push_back(obj);
		}

    	if(elementName=="random")
		{
		  float minX,maxX,minY,maxY;
		  pElem->QueryFloatAttribute("minX",&minX);
		  pElem->QueryFloatAttribute("maxX",&maxX);
		  pElem->QueryFloatAttribute("minY",&minY);
		  pElem->QueryFloatAttribute("maxY",&maxY);	

		  int number;
		  pElem->QueryIntAttribute("number", &number);

			for(int i = 0; i < number; i++)
			{
				float x = ((rand()%100)/100.0f) * (maxX-minX) + minX; 
				float y = ((rand()%100)/100.0f) * (maxY-minY) + minY; 
				DetectedObject obj;
				obj.SetX(x);
				obj.SetY(y);		  
				objects.push_back(obj);
			}
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

		  vehicle1 = Vehicle(halfWidth, leftWheelConstant, rightWheelConstant, field_of_view, max_range, wheelRadius, wheelBase);
		  vehicle1.SetLocation(x,y,theta);	
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

		  vehicle2 = Vehicle(halfWidth, leftWheelConstant, rightWheelConstant, field_of_view, max_range, wheelRadius, wheelBase);
		  vehicle2.SetLocation(x,y,theta);	
		}
    }

  return(true);	
}

void Environment::OnReceiveVehicle1(){
	float x,y,theta;
	vehicle1.UpdateLocation(vehicle1_ticks.GetLeftWheel(), vehicle1_ticks.GetRightWheel());
	vehicle1.GetLocation(x,y,theta);
	vehicle1_location.SetX(x);
	vehicle1_location.SetY(y);
	vehicle1_location.SetHeading(theta);
	vehicle1_location.SetFlagged(true);


	printf("Vehicle 1 L: %i| R: %i| |\n", vehicle1_ticks.GetLeftWheel(), vehicle1_ticks.GetRightWheel());

	printf("Vehicle 1 X: %f| Y: %f| HEADING: %f|\n", x,y,theta);

	
	float detectX, detectY, detectSqDist;
	bool detectedObjs = vehicle1.Detect(objects, detectX, detectY, detectSqDist);
	bool detectedLines = vehicle1.Detect(lines, detectX, detectY, detectSqDist, detectSqDist);
	bool detectedVehicle2 = vehicle1.Detect(vehicle2, detectX, detectY, detectSqDist, detectSqDist);

	if(detectedObjs || detectedLines || detectedVehicle2){
		float local_detectX, local_detectY;
		vehicle1.TransformInverse(detectX, detectY, local_detectX, local_detectY);
		vehicle1_detection.SetX(local_detectX);
		vehicle1_detection.SetY(local_detectY);
		vehicle1_detection.SetRange(sqrt(detectSqDist));
		vehicle1_detection.SetFlagged(true);
	}
}

void Environment::OnReceiveVehicle2(){
	float x,y,theta;
	vehicle2.UpdateLocation(vehicle2_ticks.GetLeftWheel(), vehicle2_ticks.GetRightWheel());
	vehicle2.GetLocation(x,y,theta);
	vehicle2_location.SetX(x);
	vehicle2_location.SetY(y);
	vehicle2_location.SetHeading(theta);
	vehicle2_location.SetFlagged(true);

	printf("Vehicle 2 X: %f| Y: %f| HEADING: %f|\n", x,y,theta);

	float detectX, detectY, detectSqDist;
	bool detectedObjs = vehicle2.Detect(objects, detectX, detectY, detectSqDist);
	bool detectedLines = vehicle2.Detect(lines, detectX, detectY, detectSqDist, detectSqDist);
	bool detectedVehicle1 = vehicle2.Detect(vehicle1, detectX, detectY, detectSqDist, detectSqDist);

	if(detectedObjs || detectedLines || detectedVehicle1){
		float local_detectX, local_detectY;
		vehicle2.TransformInverse(detectX, detectY, local_detectX, local_detectY);
		vehicle2_detection.SetX(local_detectX);
		vehicle2_detection.SetY(local_detectY);
		vehicle2_detection.SetRange(sqrt(detectSqDist));
		vehicle2_detection.SetFlagged(true);
	}
}


void Environment::Process()
{
 	// ---- Termination Signal ------ //
  	if (signal (SIGINT, termination_handler) == SIG_IGN)
    	signal (SIGINT, SIG_IGN);

    // Iterate and send out information about obstacles within environment
    static int object_ID = 0;
    if(object_ID < objects.size())
    {
    	printf("Publish Obstacle - %i\n", object_ID);
    	obstacle.SetX(objects[object_ID].GetX());
    	obstacle.SetY(objects[object_ID].GetY());
    	obstacle.SetFlagged(true);
    	object_ID++;
    }

    if (ElapsedTime() >= terminationTime) // termination when time elapsed
    	signal (SIGINT, SIG_IGN);

    // printf("demo4_virtual_env\n");

	// vehicle_location_copy = vehicle_location;
	// vehicle_location2_copy.SetFlagged(true);
	// vehicle_location2_copy = vehicle_location2;
	// vehicle_location_copy.SetFlagged(true);
 //    float elapsedTime = ElapsedTime();
 //    if(elapsedTime > update_interval && recv_location)
 //    {
 //    	DetectObject();
 //    	//printf("ElapsedTime in IF: %f\n",elapsedTime);
 //    	ResetClock();

 //    	recv_location = false;
 //	   }
}

