#include "ros/ros.h"
#include "std_msgs/String.h"
using namespace ros;
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
//  ROS_INFO("I got a message");
}


int main (int argc, char **argv)
{
	init(argc, argv, "listener");
	NodeHandle n;
	ROS_INFO("I'm alive! I feel happy! oh so happy!");
	Subscriber sub = n.subscribe("IMU_data", 1000, chatterCallback);
	spin();
	
	return 0;
}
