#pragma once

#include "SerialObject.h"

class WheelAction : public SerialObject
{
	int _leftWheel;
	int _rightWheel;	
public:
	WheelAction() { _leftWheel = 0; _rightWheel = 0; }

	int GetLeftWheel() { return(_leftWheel); }
	void SetLeftWheel(float left){ _leftWheel=left; }
	int GetRightWheel() { return(_rightWheel); }
	void SetRightWheel(float right){ _rightWheel=right; }

	virtual void Serialize(char* outBuffer);
	virtual void Deserialize(const char* inBuffer);
	virtual int GetObjectSize();
};