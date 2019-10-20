#ifndef WHEELSMESSAGE_H
#define WHEELSMESSAGE_H

#include "Message.h"
#include <iostream>

class WheelsMessage : public Message
{
public:
    WheelsMessage();
    void SetLeftWheel(int leftWheel);
    void SetRightWheel(int rightWheel);
    int GetLeftWheel();
    int GetRightWheel();

    void Serialize(int* dataBuf);
    void DeSerialize(int * dataBuf);
    int GetSize(); 
	void printData(); 
    Message * Clone(); 
private:
    int _leftWheel;
	int _rightWheel;
};

#endif