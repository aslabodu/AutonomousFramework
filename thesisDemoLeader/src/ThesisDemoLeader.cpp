#include "ThesisDemoLeader.h"

// ------------------------------------------
// IMPORTANT!! - Make sure there is a definition for CreateApplicationNode()
Node* CreateApplicationNode()
{
	return new ThesisDemoLeader();        // Make sure to change this to correct Node class type
}



Message * getCorrectMsg(int typeId)
{
    CommandMessage * msg1; 
	WheelsMessage * msg2;
	switch(typeId)
	{
		case 0:
			msg1 = new CommandMessage(); 
			return msg1;
			break;
		case 1:
			msg2 = new WheelsMessage(); 
			return msg2;
			break;
	}
}
// ------------------------------------------

void termination_handler (int signum)
{
  Node::Get()->Terminate(); // Example call to terminate the application with OS control signal
}

void ThesisDemoLeader::Setup(int argc, char** argv)
{
    //std::string output1;
    //std::string output2;
    //output1 = FindTopicName("PLAN_WHEELS_OUTPUT");
    //output2 = FindTopicName("PLAN_COMMAND_OUTPUT");

    // Allocate Message Memory //
    _command = new CommandMessage(); 
    _wheels = new WheelsMessage(); 

    // Set function pointer to messages // 
    Message * (*fcnPtr)(int);
	fcnPtr = getCorrectMsg;
    setMsgFcnPtr(fcnPtr);

    //Publish(output1, &_wheels);
    //Publish(output2, &_command);

    RegisterInitFunction(static_cast<NodeFuncPtr>(&ThesisDemoLeader::AppInit));
    
	RegisterCoreFunction(static_cast<NodeFuncPtr>(&ThesisDemoLeader::Process));
    
    RegisterExitFunction(static_cast<NodeFuncPtr>(&ThesisDemoLeader::OnExit));
}

void ThesisDemoLeader::SetNodeName(int argc, char** argv, std::string& nodeName)
{
	nodeName = "Planner";
}

void ThesisDemoLeader::AppInit()
{
    _wheels->SetLeftWheel(0);
    _wheels->SetRightWheel(0);

    // Set command message to pause state
    _command->SetCommand(2);

    timer = 0; 

}

void ThesisDemoLeader::Process()
{
    if (signal (SIGINT, termination_handler) == SIG_IGN)
    	signal (SIGINT, SIG_IGN);	

    if(timer < 100)
    {
        // go straight
        _command->SetCommand(0);
        _wheels->SetRightWheel(1);
        _wheels->SetLeftWheel(1);
        SendMessage("All", _command);
        SendMessage("VirtualEnv", _wheels);
        timer++;
    }
    else if(timer >=100 && timer <= 50)
    {
        _command->SetCommand(1);
        _wheels->SetRightWheel(1);
        _wheels->SetLeftWheel(1);
        SendMessage("All", _command);
        SendMessage("VirtualEnv", _wheels);
        if(timer == 50)
            timer = 0; 
    }

    
}

void ThesisDemoLeader::OnExit()
{

}