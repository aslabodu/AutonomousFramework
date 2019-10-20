#include "Planner.h"
#include "Clock.h"
#include "Keyboard.h"

#include <cmath>
#include <cstdio>
#include <signal.h>
#include <unistd.h>

#include "Distribution.h"

// ------------------------------------------
// IMPORTANT!! - Make sure there is a definition for CreateApplicationNode()
Node* CreateApplicationNode()
{
	return new Planner();        // Make sure to change this to correct Node class type
}
// ------------------------------------------


void termination_handler (int signum)
{
  Node::Get()->Terminate(); // Example call to terminate the application with OS control signal
}


void Planner::Setup(int argc, char** argv)
{
	std::string input1;
	std::string input2;
	std::string output1;

	input1 = FindTopicName("PLAN_DETECTION_INPUT");
	output1 = FindTopicName("PLAN_WHEELS_OUTPUT");

	Subscribe(input1, &detected_object);    
	RegisterInputFunction(input1,static_cast<NodeFuncPtr>(&Planner::OnReceiveObject));	

	Publish(output1, &wheels);
    
	RegisterInitFunction(static_cast<NodeFuncPtr>(&Planner::AppInit));
    
	RegisterCoreFunction(static_cast<NodeFuncPtr>(&Planner::Process));
    
    RegisterExitFunction(static_cast<NodeFuncPtr>(&Planner::OnExit));
}

void Planner::SetNodeName(int argc, char** argv, std::string& nodeName)
{
	nodeName = "Planner";
}

void Planner::AppInit()
{
	// --- Initially set to stop
	wheels.SetLeftWheel(0);
	wheels.SetRightWheel(0);
	wheels.SetFlagged(true);
	printf("Starting at Pause State......\n");

	recv_object = false;
	//steering_timer = FLT_MAX;
	steering_timer = 0.0f;
	detection_timer = 0.0f;
	ResetClock();
	halt = false;

	// Init = true;
	Forward = false;
	Pause = true;
	Turn = false;
	// srand (time(NULL));

	// init_timer = 25.0f;
	detection_timer = 25.0f;
}


void Planner::OnReceiveObject()
{
	if(Turn == false)
	{
		recv_object = true;
	}
	// printf("OBJECT RECEIVED: rel_x: %f| rel_y: %f|\n", detected_object.GetX(), detected_object.GetY());
}

void Planner::Process()
{
	static Uniform unif(1.0, 5.0);

 	// Example handle termination signal CTRL-C --- Call "termination_handler"
  	if (signal (SIGINT, termination_handler) == SIG_IGN)
    	signal (SIGINT, SIG_IGN);

	if(recv_object && (Forward || Pause))
    {
		if(rand() % 100 < 50)		// random turn
		{
			wheels.SetLeftWheel(-1);
			wheels.SetRightWheel(1);
			wheels.SetFlagged(true);
			printf("Turn State: Left......\n");				
		}
		else
		{
			wheels.SetLeftWheel(1);
			wheels.SetRightWheel(-1);
			wheels.SetFlagged(true);
			printf("Turn State: Right......\n");									
		}

		Turn = true;
		Forward = false;
		steering_timer = unif.GetRV();
		detection_timer = steering_timer + 1.0;
		ResetClock();

		recv_object = false;
    }
    else if(Turn && ElapsedTime() > steering_timer)
    {
    	wheels.SetLeftWheel(0);
		wheels.SetRightWheel(0);
		wheels.SetFlagged(true);
		printf("Pause State......\n");

		Turn = false;
		Pause = true;
    }
    else if(Pause && ElapsedTime() > detection_timer)
    {
    	wheels.SetLeftWheel(1);
		wheels.SetRightWheel(1);
		wheels.SetFlagged(true);
		printf("Forward State......\n");

		Forward = true;
		Pause = false;
    }


    return; // ------------------------------------------------------------------------

  	
    if(Forward)
    {
    	wheels.SetLeftWheel(1);
		wheels.SetRightWheel(1);
		wheels.SetFlagged(true);
		printf("Go Forward......\n");
    }

    if(ElapsedTime() > steering_timer)
    {
    	steering_timer = FLT_MAX;
    	if (!recv_object)
    	{
	    	wheels.SetLeftWheel(0);
			wheels.SetRightWheel(0);
			wheels.SetFlagged(true);
			printf("Stop......\n");
			Forward=true;
		}
    }


    if(recv_object)
    {
    	//if(steering_timer == FLT_MAX)
    	if(ElapsedTime() > detection_timer)
    	{
	    	float obj_x = detected_object.GetX();
	    	float obj_y = detected_object.GetY();
			float sqDist = ((obj_x) * (obj_x) + (obj_y) * (obj_y));
			float c = cos(90.0f * M_PI / 180.0f);
			float s = sin(90.0f * M_PI / 180.0f);
			float a = (obj_x) / sqrtf(sqDist);
			float b = (obj_y) / sqrtf(sqDist);
			float dot = (c*b) - (s*a);	// actually cross-product			

			// printf("Dot %f\n", dot);
			// printf("Dot %f|%f|%f|%f|%f|%f|\n", sqDist, c, s, a, b, dot);

			// if(dot < 0)	// object is to the right
			// {
			// 	wheels.SetLeftWheel(-1);
			// 	wheels.SetRightWheel(1);
			// 	wheels.SetFlagged(true);
			// 	printf("Turning Left......\n");
			// }
			// else if(dot > 0) // object is to the left
			// {
			// 	wheels.SetLeftWheel(1);
			// 	wheels.SetRightWheel(-1);
			// 	wheels.SetFlagged(true);
			// 	printf("Turning Right......\n");			
			// }
			// else
			// {
				if(rand() % 100 < 50)		// random turn
				{
					wheels.SetLeftWheel(-1);
					wheels.SetRightWheel(1);
					wheels.SetFlagged(true);
					printf("Turning Left......\n");				
				}
				else
				{
					wheels.SetLeftWheel(1);
					wheels.SetRightWheel(-1);
					wheels.SetFlagged(true);
					printf("Turning Right......\n");									
				}
			// }

			// Set timer to reset steering
			// steering_timer = 3.0f;
			steering_timer = unif.GetRV();
			detection_timer = steering_timer + 1.0;
			Forward = false;
			ResetClock();
		}

    	recv_object = false;
    }



    Keyboard_Update(0, 1000);
    switch(Keyboard_GetLastKey()){
    	case 'q':	// Stop (Kill Switch)
	    	wheels.SetLeftWheel(0);
			wheels.SetRightWheel(0);
			wheels.SetFlagged(true);
			halt = true;
			printf("HALT!!!!!\n");
			break;
		case 'w': 	// Resume (Take kill switch)
			halt = false;
			printf("Resume.....\n");
			break;
    }

}


void Planner::OnExit()
{
}




