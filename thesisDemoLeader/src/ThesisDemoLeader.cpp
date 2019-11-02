#include "ThesisDemoLeader.h"
#include <chrono>
#include <ctime>

// ------------------------------------------
// IMPORTANT!! - Make sure there is a definition for CreateApplicationNode()
Node* CreateApplicationNode()
{
	return new ThesisDemoLeader();        // Make sure to change this to correct Node class type
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

    //Publish(output1, &_wheels);
    //Publish(output2, &_command);

    RegisterInitFunction(static_cast<NodeFuncPtr>(&ThesisDemoLeader::AppInit));
    
	  RegisterCoreFunction(static_cast<NodeFuncPtr>(&ThesisDemoLeader::Process));
    
    RegisterExitFunction(static_cast<NodeFuncPtr>(&ThesisDemoLeader::OnExit));
}

void ThesisDemoLeader::SetNodeName(int argc, char** argv, std::string& nodeName)
{
	nodeName = "PLAN1";
}

void ThesisDemoLeader::AppInit()
{
    sleep(3);
  //  ros::Rate rate(24.);
    _wheels->SetLeftWheel(0);
    _wheels->SetRightWheel(0);

    // Set command message to pause state
    //_command->SetCommand(2);

    timer = 0; 

}

void ThesisDemoLeader::Process()
{
    if (signal (SIGINT, termination_handler) == SIG_IGN)
    	signal (SIGINT, SIG_IGN);	
  
    bool turn; 
    // Send go straight command // 
    if(timer == 0)
    {
        _command->SetCommand(0);
        SendMessage("OldLeathrum", _command);
        // Send wheel values ot environment
        _wheels->SetRightWheel(1);
        _wheels->SetLeftWheel(1);
        SendMessage("VirtualEnv", _wheels);
        
        turn = false; 
    }

    

      if(timer < 100100)
      {
          // go straight
          _wheels->SetRightWheel(1);
          _wheels->SetLeftWheel(1);
          SendMessage("VirtualEnv", _wheels);
          timer++;
      }
      else if( timer >= 100100 && timer < 106146)
      {
          if( turn == false)
          {
            turn = true;
            _command->SetCommand(1);
            SendMessage("OldLeathrum", _command);
          }
          _wheels->SetRightWheel(10);
          _wheels->SetLeftWheel(-10);
          SendMessage("VirtualEnv", _wheels);
          timer++; 
      }
      else if(timer >= 106145 && timer < 117145)
      {
          if(turn == true)
          {
            turn = false;
            _command->SetCommand(0);
            SendMessage("OldLeathrum", _command);
          }

          _wheels->SetRightWheel(1);
          _wheels->SetLeftWheel(1);
          SendMessage("VirtualEnv", _wheels);
          timer++;
      }
      else if(timer >= 117145 && timer < 123145)
      {
          if(turn == false)
          {
            turn = true;
            _command->SetCommand(1);
            SendMessage("OldLeathrum", _command);
          }
          _wheels->SetRightWheel(10);
          _wheels->SetLeftWheel(-10);
          SendMessage("VirtualEnv", _wheels);
          timer++; 
          
          if(timer == 123145)
            timer=0; 
      }

}

void ThesisDemoLeader::OnExit()
{

}
