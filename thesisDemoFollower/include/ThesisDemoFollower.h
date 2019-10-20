#ifndef THESISDEMOFOLLOWER_H
#define THESISDEMOFOLLOWER_H

#include "Node.h"
#include "CommandMessage.h"
#include "WheelsMessage.h"
#include <signal.h>

Message * getCorrectMsg(int typeId);

class ThesisDemoFollower : public Node
{

public:

protected:
    void Setup(int argc, char** argv);	
	void SetNodeName(int argc, char** argv, std::string& nodeName);
private:
    void Process();
    void AppInit();
    void OnExit();

    int timer;
    WheelsMessage * _wheels;
    CommandMessage * _command; 
};

#endif