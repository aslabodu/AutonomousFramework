#include <ros/ros.h>
#include "LocationObject.h"
#include <visualization_msgs/Marker.h>
#include "std_msgs/ByteMultiArray.h"

bool _publish;
LocationObject * vehicle_location; 
visualization_msgs::Marker rover1;
double x,y, heading; 

void OnReceiveLocation(const std_msgs::ByteMultiArray::ConstPtr& msg)
{
    std::cout << "I am in the publisher " << std::endl; 
    vehicle_location = new LocationObject(); 
    char * recvBuffer = new char[vehicle_location->GetObjectSize()];
    for(int i = 0; i < vehicle_location->GetObjectSize(); i++)	// Convert msg vector to char array
	  recvBuffer[i] = (char)msg->data[i];

    vehicle_location->Deserialize(recvBuffer);	// Deserialize data

	printf("Received Location:  X:%f| Y:%f| Theta:%f \n", vehicle_location->GetX(), vehicle_location->GetY(), vehicle_location->GetHeading());
	x = vehicle_location->GetX();
    y = vehicle_location->GetY();
    heading = vehicle_location->GetHeading();

    /* 

        rover1.header.frame_id = "/my_frame";
        rover1.header.stamp = ros::Time::now();

        rover1.ns = "basic_shapes";
        rover1.id = 0;

        rover1.type = visualization_msgs::Marker::ARROW;
        rover1.action = visualization_msgs::Marker::ADD;
        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    // %Tag(POSE)%
        rover1.pose.position.x = vehicle_location->GetX();
        rover1.pose.position.y = vehicle_location->GetY();
        rover1.pose.position.z = 0;
        rover1.pose.orientation.x = vehicle_location->GetHeading();
        rover1.pose.orientation.y = 0.0;
        rover1.pose.orientation.z = 0.0;
        rover1.pose.orientation.w = 1.0;
    // %EndTag(POSE)%

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
    // %Tag(SCALE)%
        rover1.scale.x = 1.0;
        rover1.scale.y = 1.0;
        rover1.scale.z = 1.0;
    // %EndTag(SCALE)%

        // Set the color -- be sure to set alpha to something non-zero!
    // %Tag(COLOR)%
        rover1.color.r = 0.0f;
        rover1.color.g = 1.0f;
        rover1.color.b = 0.0f;
        rover1.color.a = 1.0;
    // %EndTag(COLOR)%

    // %Tag(LIFETIME)%
        rover1.lifetime = ros::Duration();
    // %EndTag(LIFETIME)%
*/
    _publish = true; 

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "viz");
    ros::NodeHandle n;
   // ros::Rate r(20);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1); 
    _publish = false;
    ros::Subscriber sub = n.subscribe("/TRUTH_ROVER_LOCATION", 1000, OnReceiveLocation);

     while (ros::ok())
    {
        while (marker_pub.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        if(_publish == true)
        {
            _publish= false; 
            rover1.header.frame_id = "/my_frame";
            rover1.frame_locked = true; 
        rover1.header.stamp = ros::Time::now();

        rover1.ns = "basic_shapes";
        rover1.id = 0;

        rover1.type = visualization_msgs::Marker::ARROW;
        rover1.action = visualization_msgs::Marker::ADD;
        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        // %Tag(POSE)%

        rover1.pose.position.x = vehicle_location->GetX();
        rover1.pose.position.y = vehicle_location->GetY();
        rover1.pose.position.z = 0;
        rover1.pose.orientation.x = 0;
        rover1.pose.orientation.y = 0;
        rover1.pose.orientation.z = sin((vehicle_location->GetHeading() * (M_PI/180))/2);
        rover1.pose.orientation.w = cos((vehicle_location->GetHeading() * (M_PI/180))/2);
        // %EndTag(POSE)%

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        // %Tag(SCALE)%
        rover1.scale.x = 10.0;
        rover1.scale.y = 10.0;
        rover1.scale.z = 10.0;
        // %EndTag(SCALE)%

        // Set the color -- be sure to set alpha to something non-zero!
        // %Tag(COLOR)%
        rover1.color.r = 0.0f;
        rover1.color.g = 1.0f;
        rover1.color.b = 0.0f;
        rover1.color.a = 1.0;
    // %EndTag(COLOR)%

    // %Tag(LIFETIME)%
        rover1.lifetime = ros::Duration();
    // %EndTag(LIFETIME)%
            marker_pub.publish(rover1);
        }
     //   r.sleep();
        ros::spinOnce(); 
    }
    return 0; 
}