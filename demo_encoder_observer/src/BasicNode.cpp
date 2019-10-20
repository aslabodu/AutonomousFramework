#include "BasicNode.h"

#include <cmath>
#include <cstdio>
#include <signal.h>
#include <unistd.h>

#include <cstring>

#include "Keyboard.h"

// ------------------------------------------
// IMPORTANT!! - Make sure there is a definition for CreateApplicationNode()
Node* CreateApplicationNode()
{
	return new BasicNode();        // Make sure to change this to correct Node class type
}
// ------------------------------------------


void termination_handler (int signum)
{
  Node::Get()->Terminate(); // Example call to terminate the application with OS control signal
}


void BasicNode::Setup(int argc, char** argv)
{
    std::string input_topic = FindTopicName("input1");

    // Example of subscribing to certain topic and connecting to object "input".    
	Subscribe(input_topic, &encoder_ticks);

    // Also example registing "OnReceiveInput" to be an input function called to handle received data from topic
	RegisterInputFunction(input_topic,static_cast<NodeFuncPtr>(&BasicNode::OnReceiveInput));	

    // Example of registering "AppInit" to be an initialization function processed once after Setup()
	RegisterInitFunction(static_cast<NodeFuncPtr>(&BasicNode::AppInit));

    RegisterCoreFunction(static_cast<NodeFuncPtr>(&BasicNode::Process));

    // Example of registering "OnExit" to be an exit function called before application exit
    RegisterExitFunction(static_cast<NodeFuncPtr>(&BasicNode::OnExit));
}

void BasicNode::SetNodeName(int argc, char** argv, std::string& nodeName)
{
	nodeName = "EncoderObserver";
}

void BasicNode::AppInit()
{	
    // Example application specific initialization 

    enabled = false;

    Keyboard_Init();
}


void BasicNode::OnReceiveInput()
{
    printf("L: %i | R: %i\n", encoder_ticks.GetLeftWheel(), encoder_ticks.GetRightWheel());
    if(enabled){
        leftWheelData.push_back(encoder_ticks.GetLeftWheel());
        rightWheelData.push_back(encoder_ticks.GetRightWheel());
    }
}

void BasicNode::Process()
{
 	// Example handle termination signal CTRL-C --- Call "termination_handler"
  	if (signal (SIGINT, termination_handler) == SIG_IGN)
    	signal (SIGINT, SIG_IGN);	

    Keyboard_Update(0, 1000);

    if (Keyboard_GetLastKey() == 'p'){
        enabled = true;
    }
    else if(Keyboard_GetLastKey() == 's'){
        if(enabled){
            enabled = false;
            SaveDataToFile();
            leftWheelData.clear();
            rightWheelData.clear();
        }
    }
}


void BasicNode::OnExit()
{
    // Example handling application exit....
    printf("Node Finished.......\n");

    Keyboard_Cleanup();
}


void BasicNode::SaveDataToFile(){

    printf("Enter filename to write output: \n");
    char filename [256];
    scanf("%255s", filename);
    char path[256];
    strcat(strcpy(path, getenv("HOME")), "//");
    strcat(path, filename);

    FILE * pfile = fopen(path, "w");

    if(pfile != NULL)
    {
        for(unsigned int i = 0; i < leftWheelData.size(); i++)
        {
            fprintf(pfile, "%i\t%i\n", leftWheelData[i], rightWheelData[i]);
        }

        fclose(pfile);
        printf("Saving Done....\n");
    }
}
