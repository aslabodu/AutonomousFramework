#include "CommandMessage.h"

CommandMessage::CommandMessage()
{
    _command = -1; 
    SetMsgType(0);
}

int CommandMessage::GetCommand()
{
    return _command; 
} 

void CommandMessage::SetCommand(int command)
{
    _command = command; 
}

void CommandMessage::Serialize(int* dataBuf)
{
    int index = 0;
	int * dataRef;

	dataRef = (int*)&_command;
	for (int i = 0; i < sizeof(_command) / sizeof(char); i++)
		dataBuf[index++] = dataRef[i];
}

void CommandMessage::DeSerialize(int* dataBuf)
{
    int index = 0;
	int *dataRef;

	dataRef = (int*)&_command;
	for (int i = 0; i < sizeof(_command) / sizeof(char); i++)
		dataRef[i] = dataBuf[index++];
}

int CommandMessage::GetSize()
{
    return(sizeof(_command));
}

void CommandMessage::printData()
{
    std::cout << "Command: " << _command << std::endl; 
}

Message * CommandMessage::Clone()
{
    return new CommandMessage(*this);
}