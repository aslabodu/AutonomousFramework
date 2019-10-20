#include "WheelsMessage.h"

WheelsMessage::WheelsMessage()
{
    _leftWheel = 0; 
    _rightWheel = 0; 
    SetMsgType(1);
}

void WheelsMessage::SetLeftWheel(int leftWheel)
{
    _leftWheel = leftWheel; 
}
    
void WheelsMessage::SetRightWheel(int rightWheel)
{
    _rightWheel = rightWheel;
}

int WheelsMessage::GetLeftWheel()
{
    return _leftWheel;
}

int WheelsMessage::GetRightWheel()
{
    return _rightWheel; 
}

void WheelsMessage::Serialize(int* dataBuf)
{
	int index = 0;
	int * dataRef;

	dataRef = (int*)&_leftWheel;
	for (int i = 0; i < sizeof(_leftWheel) / sizeof(int); i++)
		dataBuf[index++] = dataRef[i];

	dataRef = (int*)&_rightWheel;
	for (int i = 0; i < sizeof(_rightWheel) / sizeof(int); i++)
		dataBuf[index++] = dataRef[i];
}

void WheelsMessage::DeSerialize(int * dataBuf)
{
	int index = 0;
	int *dataRef;

	dataRef = (int*)&_leftWheel;
	for (int i = 0; i < sizeof(_leftWheel) / sizeof(int); i++)
		dataRef[i] = dataBuf[index++];

	dataRef = (int*)&_rightWheel;
	for (int i = 0; i < sizeof(_rightWheel) / sizeof(int); i++)
		dataRef[i] = dataBuf[index++];
}

int WheelsMessage::GetSize()
{
    return(
		(sizeof(_leftWheel) + 
		sizeof(_rightWheel))/ sizeof(int)
		);
} 
	
void WheelsMessage::printData()
{
    std::cout << "Left Wheel: " << _leftWheel << " " << "Right Wheel: " << _rightWheel << std::endl;
}

Message * WheelsMessage::Clone()
{
    return new WheelsMessage(*this);
}