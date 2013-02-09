#include "ros/ros.h"
#include <IMU/imu_filter.h>

using namespace std;

int main(int argc, char **argv)	{

	ros::init(argc, argv, "calculate_orientation_client");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<IMU::imu_filter>("Calculate_Orientation");
	
	IMU::imu_filter srv;
	srv.request.rawIMU.a_x = -0.01244;
	srv.request.rawIMU.a_y = -0.00801;
	srv.request.rawIMU.a_z = .98175;
	srv.request.rawIMU.w_x = -1.88324;
	srv.request.rawIMU.w_x = -.59418;
	srv.request.rawIMU.w_x = .10736;

	srv.response.roll = 42.5;
	if(client.call(srv))	{
		ROS_INFO("Roll: %f", srv.response.roll);
	}	
	else	{
		ROS_ERROR("failed to call calculate_orientation");
	}
}
