#include "ros/ros.h"
#include <IMU/spatialRaw.h>
using namespace ros;
void chatterCallback(const IMU::spatialRaw & msg)
{
	ROS_INFO("%f", msg.w_x);
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
