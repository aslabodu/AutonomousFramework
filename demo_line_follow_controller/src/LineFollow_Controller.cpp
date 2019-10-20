#include "LineFollower_Controller.h"
#include "tinyxml2.h"
#include "Clock.h"
#include "Keyboard.h"

#include <cmath>
#include <cstdio>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>

#include <termios.h>
#include <fcntl.h>

const char* parameterFile = "../catkin_ws/src/demo_LineFollow_controller/config/parameters.xml";

// ------------------------------------------
// IMPORTANT!! - Make sure there is a definition for CreateApplicationNode()
Node* CreateApplicationNode()
{
	return new LineFollwer_Controller();        // Make sure to change this to correct Node class type
}
// ------------------------------------------


void termination_handler (int signum)
{
  Node::Get()->Terminate(); // Example call to terminate the application with OS control signal
}


void LineFollwer_Controller::Setup(int argc, char** argv)
{

	std::string input1 = FindTopicName("input1");
	std::string output1 = FindTopicName("output1");

	Publish(output1, &roverLocation);
	Subscribe(input1, &roverWheels);

	RegisterInitFunction(static_cast<NodeFuncPtr>(&LineFollwer_Controller::AppInit));

	RegisterInputFunction(input1, static_cast<NodeFuncPtr>(&LineFollwer_Controller::OnWheels));
    
	RegisterCoreFunction(static_cast<NodeFuncPtr>(&LineFollwer_Controller::Process));
    
    RegisterExitFunction(static_cast<NodeFuncPtr>(&LineFollwer_Controller::OnExit));
}

void LineFollwer_Controller::SetNodeName(int argc, char** argv, std::string& nodeName)
{
	nodeName = "LineFollwer_Controller";
}

void LineFollwer_Controller::AppInit()
{
	if(Load(parameterFile) == false)
    {
    	printf("Failed to Load Environment File: %s\n", parameterFile);
    	Node::Get()->Terminate();
    }
	roverLocation.SetX(0);
	roverLocation.SetY(0);
	roverLocation.SetHeading(referenceHead);
	roverLocation.SetFlagged(true);
}


bool LineFollwer_Controller::Load(const char* filename)
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
			pElem->QueryIntAttribute("N",&N);
			pElem->QueryFloatAttribute("roverWidth",&roverWidth);
			pElem->QueryFloatAttribute("full_rotation_speed",&full_rotation_speed);
			pElem->QueryFloatAttribute("full_translation_speed",&full_translation_speed);
			pElem->QueryFloatAttribute("Heading",&referenceHead);

		}
    }

  return(true);	
}

void LineFollwer_Controller::Process()
{
 	// ---- Termination Signal ------ //
  	if (signal (SIGINT, termination_handler) == SIG_IGN)
    	signal (SIGINT, SIG_IGN);

	//printf("%f \t %f \t %f\n", roverLocation.GetX(), roverLocation.GetY(), roverLocation.GetHeading());
	float elapsedTime = ElapsedTime();
    if(elapsedTime > update_interval)
    {
    	//printf("LeftWheel %f| RightWheel %f|\n",roverWheels.GetLeftWheel(),roverWheels.GetRightWheel());
    	//printf("X: %f| Y: %f| HEADING: %f|\n", roverLocation.GetX(), roverLocation.GetY(), roverLocation.GetHeading());
    	Rotate(elapsedTime);
    	Translate(elapsedTime);    	
    	ResetClock();    	
	}
}


void LineFollwer_Controller::Rotate(double elapsedTime)
{
	float theta = roverLocation.GetHeading();
	float x = roverLocation.GetX();
	float y = roverLocation.GetY();	
	float Hw = roverWidth/2.0f;

	if(leftWheel != rightWheel)
	{
		int direction = (leftWheel > rightWheel) ? -1 : 1;

		float rotX;
		float rotY;

		// Distance 

		if (leftWheel != 0 && rightWheel ==0){
			rotX = roverWidth/2.0f;
			rotY = 0.0f;
			if (leftWheel > 0)
				direction = -1;
			else 
				direction = 1;
		}
		else if(rightWheel != 0 && leftWheel == 0){
			rotX = -roverWidth/2.0f;
			rotY = 0.0f;
			if (rightWheel > 0)
				direction = 1;
			else 
				direction = -1;
		}
		else {
			rotX = Hw/N*(leftWheel-rightWheel);
			if(leftWheel > rightWheel)
				direction = -1;
			else 
				direction = 1;
		}

		// float speed = (abs(leftWheel - rightWheel) > 1) ? full_rotation_speed : full_rotation_speed / 2.0f;
		float speed = full_rotation_speed;
		float angleToApply = abs(leftWheel-rightWheel)*speed * elapsedTime * direction;


		// Transform axis into world coordinates....
		float c = cos(theta * M_PI / 180.0f);
		float s = sin(theta * M_PI / 180.0f);
		float t_rotX = c * rotX - s * rotY;
		float t_rotY = s * rotX + c * rotY;
		t_rotX += x;
		t_rotY += y;

		// --- Apply Rotation to Theta and X, Y coordinates --- //

		// angle to apply		
		c = cos(angleToApply * M_PI / 180.0f);
		s = sin(angleToApply * M_PI / 180.0f);

		// transform to origin
		x = x - t_rotX;
		y = y - t_rotY;

		// apply rotation
		float tx = c * x - s * y;
		float ty = s * x + c * y;

		// transform back
		x = tx + t_rotX;
		y = ty + t_rotY;

		float PHI = theta + angleToApply;

		if (PHI < 0)
			PHI += 360;
		else if (PHI > 360)
			PHI -= 360;

		// update orientation
		roverLocation.SetHeading(PHI);
		roverLocation.SetFlagged(true);

		// update location
		roverLocation.SetX(x);
		roverLocation.SetY(y);
		roverLocation.SetFlagged(true);
	}


}


void LineFollwer_Controller::Translate(double elapsedTime)
{
	float theta = roverLocation.GetHeading();
	float x = roverLocation.GetX();
	float y = roverLocation.GetY();

	if (leftWheel != 0 && rightWheel != 0 && leftWheel != (-rightWheel))
	{
		float moveVec[2];
		moveVec[0] = cos(theta * M_PI / 180.0f);
		moveVec[1] = sin(theta * M_PI / 180.0f);

		float speed = 0;

		if (abs(rightWheel) > abs(leftWheel))
			speed = rightWheel*full_translation_speed;
		else 
			speed = leftWheel*full_translation_speed;


		x = x + moveVec[0] * speed * elapsedTime;
		y = y + moveVec[1] * speed * elapsedTime;

		// update location
		roverLocation.SetX(x);
		roverLocation.SetY(y);
		roverLocation.SetFlagged(true);

	}
}

// Step of 3 (Might need to change)
void LineFollwer_Controller::OnWheels(){
	// // Converting PWM to Range
	leftWheel = 3000000;
	rightWheel = 3000000;
	int LS, RS;
	// Leftside
	for (int k = -N; k <= N; k++){
		LS = 63+63/N*k; // the step PWM calculations
		if (LS == 0) {LS++;} // PWM cannot equal 0
		if (LS >= roverWheels.GetLeftWheel()) {leftWheel = k;break;}
	}
	// rightside
	for (int k = -N; k <= N; k++){
		RS = 191 + 63/N*k; // the step PWM calculations
		if (RS >= roverWheels.GetRightWheel()) {rightWheel = k; break;}	
	}
	//printf("\tLS = %i\t RS = %i\n", roverWheels.GetLeftWheel(),roverWheels.GetRightWheel());
	printf("\tLS = %i\t RS = %i\n", leftWheel,rightWheel);
}

void LineFollwer_Controller::OnExit()
{
	Keyboard_Cleanup();
}