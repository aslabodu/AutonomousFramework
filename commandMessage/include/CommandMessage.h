#ifndef COMMANDMESSAGE_H
#define COMMANDMESSAGE_H

#include "Message.h"
#include<iostream>

class CommandMessage : public Message 
{
public:
    CommandMessage();
    int GetCommand(); 
    void SetCommand(int command);
    void Serialize(int* dataBuf);
    void DeSerialize(int * dataBuf);
    int GetSize(); 
	void printData();
    Message * Clone();
private:
    int _command;                   // 0 = full forward
                                    // 1 = turn left 90 degrees
                                    // 2 = stop movement
};

#endif

