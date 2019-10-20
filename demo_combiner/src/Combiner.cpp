#include "Combiner.h"
#include "SerialObject.h"
#include <cstdio>
#include <signal.h>
#include <unistd.h>

// termination_handler
// system termination signal callback.  Makes call to Node::Terminate() to halt node's main loop and exit application.
void termination_handler (int signum)
{
  Node::Get()->Terminate();
}


void Combiner::Setup(int argc, char** argv)
{
	CreateObjects(argc, argv, _physicalObject, _virtualObject, _outputObject);
	SetTopicNames(argc, argv, _physicalName, _virtualName, _outputName);
	SetMode(argc, argv, _mode);

	Subscribe(_physicalName, _physicalObject);
	Subscribe(_virtualName, _virtualObject);
	Publish(_outputName, _outputObject);

	RegisterInputFunction(_physicalName, static_cast<NodeFuncPtr>(&Combiner::OnReceivePhysical));
	RegisterInputFunction(_virtualName, static_cast<NodeFuncPtr>(&Combiner::OnReceiveVirtual));
	RegisterCoreFunction(static_cast<NodeFuncPtr>(&Combiner::Process));

	recv_physical = false;
	recv_virtual = false;
}

int Combiner::GetMode()
{
	return _mode;
}

void Combiner::OnReceivePhysical()
{
	recv_physical = true;
}

void Combiner::OnReceiveVirtual()
{
	recv_virtual = true;
}


// Process
// --- Waits until approapriate data has been received before calling Combine() and flagging output data for publishing.
// --- Note: Combine will be called once all data is received based on the current mode of operation.
// --- (i.e. if mode is set to 0, Combine will be called once physical data has been received.)
// --- (if mode is set to 2, Combine will be called once both physical and virtual data have been received.)
// --- Note: Standard system termination signal (CTRL-C on Unix/Linux) is checked to in order to terminate Combiner Node
void Combiner::Process()
{	
 	// ---- Termination Signal ------ //
  	if (signal (SIGINT, termination_handler) == SIG_IGN)
    	signal (SIGINT, SIG_IGN);

	if(_mode == 0 && recv_physical)
	{
		Combine(_physicalObject, _virtualObject, _outputObject);
		_outputObject->SetFlagged(true);
		recv_physical = false;
	}
	else if(_mode == 1 && recv_virtual)
	{
		Combine(_physicalObject, _virtualObject, _outputObject);
		_outputObject->SetFlagged(true);
		recv_virtual = false;
	}
	else if (_mode == 2 && recv_physical && recv_virtual)		// AND Augmented
	{
		Combine(_physicalObject, _virtualObject, _outputObject);
		_outputObject->SetFlagged(true);
		recv_physical = false;
		recv_virtual = false;
	}
	else if (_mode == 3 && (recv_physical || recv_virtual))		// OR Augmented
	{
		Combine(_physicalObject, _virtualObject, _outputObject);
		_outputObject->SetFlagged(true);
		recv_physical = false;
		recv_virtual = false;
	}
}