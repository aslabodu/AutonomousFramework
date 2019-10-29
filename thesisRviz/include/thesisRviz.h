#ifndef THESISRVIZ_H
#define THESISRVIZ_H

#include <ros/ros.h>
#include "LocationObject.h"
#include <visualization_msgs/Marker.h>
#include "std_msgs/ByteMultiArray.h"

#include <future>
#include <chrono>


class thesisRviz
{
private:
	LocationObject * vehicle_location; 
    char* recvBuffer;
    visualization_msgs::Marker rover1;
    ros::Publisher marker_pub;
    ros::Subscriber sub;
    ros::NodeHandle* nodeHandle = NULL;
	
protected:
	void Setup(int argc, char** argv);
	void SetNodeName(int argc, char** argv, std::string& nodeName);
//	void Init(int argc, char** argv);
	void OnReceiveLocation(const std_msgs::ByteMultiArray::ConstPtr& msg);
public:
    thesisRviz(int argc,char ** argv);
    void publish(); 
    bool _publish = false; 
};

#endif