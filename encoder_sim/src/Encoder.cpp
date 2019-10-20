#include "Encoder.h"

#include <cmath>
#include <cstdio>
#include <signal.h>
#include <unistd.h>
#include <time.h>

#include <cstring>

#include "Keyboard.h"
#include "Clock.h"
#include "Distribution.h"

// ------------------------------------------
// IMPORTANT!! - Make sure there is a definition for CreateApplicationNode()
Node* CreateApplicationNode()
{
	return new Encoder();        // Make sure to change this to correct Node class type
}
// ------------------------------------------


void termination_handler (int signum)
{
  Node::Get()->Terminate(); // Example call to terminate the application with OS control signal
}


void Encoder::Setup(int argc, char** argv)
{
    std::string input_topic = FindTopicName("input1");
    std::string output_topic = FindTopicName("output1");

    // Example of subscribing to certain topic and connecting to object "input".    
	Subscribe(input_topic, &wheels);

    Publish(output_topic, &encoder_ticks);

    // Example of registering "AppInit" to be an initialization function processed once after Setup()
	RegisterInitFunction(static_cast<NodeFuncPtr>(&Encoder::AppInit));

    RegisterCoreFunction(static_cast<NodeFuncPtr>(&Encoder::Process));

    // Example of registering "OnExit" to be an exit function called before application exit
    RegisterExitFunction(static_cast<NodeFuncPtr>(&Encoder::OnExit));
}

void Encoder::SetNodeName(int argc, char** argv, std::string& nodeName)
{
	nodeName = "EncoderObserver";
}

void Encoder::AppInit()
{	
    // Example application specific initialization 
    srand (time(NULL));

    ResetClock();
    Keyboard_Init();
}

void Encoder::Process()
{
 	// Example handle termination signal CTRL-C --- Call "termination_handler"
  	if (signal (SIGINT, termination_handler) == SIG_IGN)
    	signal (SIGINT, SIG_IGN);	

    float elapsedTime = ElapsedTime();
    if(elapsedTime > 0.05)
    {
        int leftWheel = wheels.GetLeftWheel();
        int rightWheel = wheels.GetRightWheel();
        //float r = rand();
        // if (leftWheel == 1 && rightWheel == 1){ // FORWARD
        //     encoder_ticks.SetRightWheel((int)(-1)*GEVinv(-0.34,-197.05,47.98,r)); // actually gamma
        //     encoder_ticks.SetLeftWheel((int)GEVinv(-0.25,235.91,49.65, r));
        // }
        // else if (leftWheel == -1 && rightWheel == -1){ // BACKWARD
        //     encoder_ticks.SetRightWheel((int)GEVinv(-0.34,-197.05,47.98,r));
        //     encoder_ticks.SetLeftWheel((int)GEVinv(-0.31,-203.12,48.20,r));
        // }
        // else if (leftWheel == -1 && rightWheel == 1){ // CCW
        //     encoder_ticks.SetRightWheel((int)LogLogisticInv(5.33,0.09,r));
        //     encoder_ticks.SetLeftWheel((int)LogisticInv(-183.89,21.40,r));
        // }
        // else if (leftWheel == 1 && rightWheel == -1){ // CW
        //     encoder_ticks.SetRightWheel((int)GEVinv(-0.29,-253.30,35.42,r));
        //     encoder_ticks.SetLeftWheel((int)GEVinv(-0.26,214.12,39.28,r));
        // }
        // else{ // STOP
        //     encoder_ticks.SetLeftWheel(0);
        //     encoder_ticks.SetRightWheel(0);
        // }
        // static Beta fl(2.93,3.95);
        // static Beta fr(3.88,6.49);
        // static Beta bl(3.15,2.7);
        // static Beta br(2.74,2.52);
        // static Beta cwl(2.85,3);
        // static Beta cwr(4.07,3.87);
        // static Beta ccwl(8.23,3.99);
        // static Beta ccwr(6.63,11.7);

        // if (leftWheel == 1 && rightWheel == 1){ // FORWARD
        //     encoder_ticks.SetRightWheel((int)(173+227*fr.GetRV()));
        //     encoder_ticks.SetLeftWheel((int)(132+288*fl.GetRV()));
        // }
        // else if (leftWheel == -1 && rightWheel == -1){ // BACKWARD
        //     encoder_ticks.SetRightWheel((int)(-305+237*br.GetRV()));
        //     encoder_ticks.SetLeftWheel((int)(-324+256*bl.GetRV()));
        // }
        // else if (leftWheel == -1 && rightWheel == 1){ // CCW
        //     encoder_ticks.SetRightWheel((int)(106+284*ccwr.GetRV()));
        //     encoder_ticks.SetLeftWheel((int)(-389+302*ccwl.GetRV()));
        // }
        // else if (leftWheel == 1 && rightWheel == -1){ // CW
        //     encoder_ticks.SetRightWheel((int)(-349+211*cwr.GetRV()));
        //     encoder_ticks.SetLeftWheel((int)(126+211*cwl.GetRV()));
        // }
        // else{ // STOP
        //     encoder_ticks.SetLeftWheel(0);
        //     encoder_ticks.SetRightWheel(0);
        // }


        static Beta fl(5.85,6.52);
        static Erlang fr(27.7,14);
        static Beta bl(4.86,1.9);
        static Beta br(4.02,1.74);
        static Erlang cwl(132,3);
        static Beta cwr(25.5,0.541);
        static Beta ccwl(3.56,1.9);
        static Normal ccwr(431.766,212.304);	// <----

        if (leftWheel == 1 && rightWheel == 1){ // FORWARD
            encoder_ticks.SetRightWheel((int)(154+fr.GetRV()));
            encoder_ticks.SetLeftWheel((int)(178+831*fl.GetRV()));
        }
        else if (leftWheel == -1 && rightWheel == -1){ // BACKWARD
            encoder_ticks.SetRightWheel((int)(-948+918*br.GetRV()));
            encoder_ticks.SetLeftWheel((int)(-1040+1010*bl.GetRV()));
        }
        else if (leftWheel == -1 && rightWheel == 1){ // CCW
            encoder_ticks.SetRightWheel((int)(ccwr.GetRV()));		// <----
            encoder_ticks.SetLeftWheel((int)(-1140+1110*ccwl.GetRV()));
        }
        else if (leftWheel == 1 && rightWheel == -1){ // CW
            encoder_ticks.SetRightWheel((int)(-1870+1860*cwr.GetRV()));
            encoder_ticks.SetLeftWheel((int)(71+cwl.GetRV()));
        }
        else{ // STOP
            encoder_ticks.SetLeftWheel(0);
            encoder_ticks.SetRightWheel(0);
        }
        printf("Left: %i | Right: %i |\n", encoder_ticks.GetLeftWheel(), encoder_ticks.GetRightWheel());

        encoder_ticks.SetFlagged(true);
        ResetClock();
    }
} 


void Encoder::OnExit()
{
    // Example handling application exit....
    printf("Node Finished.......\n");

    Keyboard_Cleanup();
}

