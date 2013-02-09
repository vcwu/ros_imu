#include "ros/ros.h"
#include <IMU/imu_filter.h>

using namespace std;

int main(int argc, char **argv)	{

	ros::init(argc, argv, "calculate_orientation_client");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<IMU::imu_filter>("Calculate_Orientation");
	
	IMU::imu_filter srv;
	if(client.call(srv))	{
		ROS_INFO("Roll: %f", srv.response.roll);
	}	
	else	{
		ROS_ERROR("failed to call calculate_orientation");
	}
}
