#include "ros/ros.h"
#include <IMU/spatialRaw.h>
#include <IMU/orientation.h>
int main()	{
	
	ros::init(argc, argv, "Publish Orientation");
	ros::NodeHandle OrNode;
	ros::Publisher OrPub = 
		OrNode.advertise<
}
