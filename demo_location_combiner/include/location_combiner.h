#pragma once

#include "Combiner.h"
#include "LocationObject.h"
#include <cmath>

#include <ros/ros.h> // For access to parameter server

class location_combiner : public Combiner
{
protected:	
	void SetNodeName(int argc, char** argv, std::string& nodeName);	
	void CreateObjects(int argc, char** argv, SerialObject*& physicalObject, SerialObject*& virtualObject, SerialObject*& outputObject);
	void SetTopicNames(int argc, char** argv, std::string& physicalName, std::string& virtualName, std::string& outputName);
	void SetMode(int argc, char**argv, int& mode);
	void Combine(SerialObject* physicalLocation, SerialObject* virtualLocation, SerialObject* outputLocation);
};