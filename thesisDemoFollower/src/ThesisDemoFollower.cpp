#include "ThesisDemoFollower.h"
#include <chrono>
#include <ctime>
#include <boost/date_time.hpp>

// ------------------------------------------
// IMPORTANT!! - Make sure there is a definition for CreateApplicationNode()
Node* CreateApplicationNode()
{
	return new ThesisDemoFollower();        // Make sure to change this to correct Node class type
}

// ------------------------------------------

void termination_handler (int signum)
{
  Node::Get()->Terminate(); // Example call to terminate the application with OS control signal
}

void ThesisDemoFollower::Setup(int argc, char** argv)
{
    //std::string output1;
    //std::string output2;
    //output1 = FindTopicName("PLAN_WHEELS_OUTPUT");
    //output2 = FindTopicName("PLAN_COMMAND_OUTPUT");

    // Allocate Message Memory //
    _command = new CommandMessage(); 
    _wheels = new WheelsMessage(); 

    // Set function pointer to messages // 

    //Publish(output1, &_wheels);
    //Publish(output2, &_command);

    RegisterInitFunction(static_cast<NodeFuncPtr>(&ThesisDemoFollower::AppInit));
    
	RegisterCoreFunction(static_cast<NodeFuncPtr>(&ThesisDemoFollower::Process));
    
    RegisterExitFunction(static_cast<NodeFuncPtr>(&ThesisDemoFollower::OnExit));
}

void ThesisDemoFollower::SetNodeName(int argc, char** argv, std::string& nodeName)
{
	nodeName = "PLAN2";
}

void ThesisDemoFollower::AppInit()
{
    _wheels->SetLeftWheel(0);
    _wheels->SetRightWheel(0);

    _prevL = 0;
    _prevR = 0; 

    timer = 0; 

}

void ThesisDemoFollower::Process()
{
    if (signal (SIGINT, termination_handler) == SIG_IGN)
    	signal (SIGINT, SIG_IGN);	

    if(CheckForMessage(0) == true)
    {
        if(GetNextMsgType(0) == 0)
        {
            _command = dynamic_cast<CommandMessage *>((GetMessage(0))->Clone()); 
            
            if(_command->GetCommand() == 0)
            {
                std::cout << "CHARLIE RECEIVED A COMMAND MESSAGE: " << 0 << "at: " 
                << boost::posix_time::second_clock::local_time().time_of_day()<< std::endl; 
                _wheels->SetRightWheel(1);
                _wheels->SetLeftWheel(1);
                SendMessage("VirtualEnv2", _wheels);
               // SendMessage("Ntiana", _command);
                _prevL = 1;
                _prevR = 1;
            }
            else if(_command->GetCommand() == 1)
            {
                std::cout << "CHARLIE RECEIVED A COMMAND MESSAGE: " << 1 << "at: " 
                << boost::posix_time::second_clock::local_time().time_of_day()<< std::endl; 
                _wheels->SetRightWheel(10);
                _wheels->SetLeftWheel(-10);
                SendMessage("VirtualEnv2", _wheels);
              //  SendMessage("Ntiana", _command);
                _prevL = 10;
                _prevR = -10;
            }
            else if(_command->GetCommand() == 2)
            {
                std::cout << "CHARLIE RECEIVED A COMMAND MESSAGE: " << 2 << "at: " 
                << boost::posix_time::second_clock::local_time().time_of_day()<< std::endl; 
                _wheels->SetLeftWheel(0);
                _wheels->SetRightWheel(0);
                SendMessage("VirtualEnv2", _wheels);
                //SendMessage("Ntiana", _command);
                _prevL = 0;
                _prevR = 0;
            }
            delete _command; 
        }
    }
    else
    {
        _wheels->SetRightWheel(_prevR);
        _wheels->SetLeftWheel(_prevL);
        SendMessage("VirtualEnv2", _wheels);
    }
}

void ThesisDemoFollower::OnExit()
{

}
