#include "Node.h"
#include "CommandMessage.h"
#include "WheelsMessage.h"
#include <cstdio>

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

int main(int argc, char** argv)
{ 
	std::cout << "YOOHOO" << std::endl; 
	Message * (*fcnPtr)(int);
	fcnPtr = getCorrectMsg;   

	Node::Get()->setMsgFcnPtr(fcnPtr);
    Node::Get()->Init(argc, argv);
	Node::Get()->Loop();

	return 0;
}
