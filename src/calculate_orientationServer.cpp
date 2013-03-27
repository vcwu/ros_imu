/******************************************************************************
* Calculate orientation SERVER
* the orientation service takes raw IMU data and converts it to a useable
*   orientation estimate using an imu filter.
*	
*
******************************************************************************/

#include "ros/ros.h"
//Message Types
#include <ros_imu/imu_filter.h>
//Our headers
#include "orientation_headers/imuFilter.h"
#include "config.h" 

IMUfilter imuFilter(seconds_from_ms(DATA_RATE), gyroscopeErrorRate);

bool calculate(ros_imu::imu_filter::Request &request, ros_imu::imu_filter::Response &response);

int main(int argc, char* argv[])
{
	
	ros::init(argc, argv, "Calculate_Orientation_server");
	ros::NodeHandle berk;
	
	imuFilter.reset();
	ros::ServiceServer service = berk.advertiseService("Calculate_Orientation", calculate );
	ROS_INFO("Ready to calculate orientation.");
	ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", deg_from_rad(imuFilter.getRoll()),
		deg_from_rad(imuFilter.getPitch()), deg_from_rad(imuFilter.getYaw()));
	ros::spin();
	return 0;
}



bool calculate(ros_imu::imu_filter::Request &request, ros_imu::imu_filter::Response &response)
{
	//Update filter with IMU data
	ros_imu::spatialRaw raw = request.rawIMU;
	ROS_INFO("Server Side, Raw Phidget Data");
	ROS_INFO("a_x: %f, a_y: %f, a_z:%f", raw.a_x, raw.a_y, raw.a_z);
	ROS_INFO("w_x: %f, w_y: %f, w_z:%f", raw.w_x, raw.w_y, raw.w_z);
	
	imuFilter.updateFilter(raw.w_x, raw.w_y, raw.w_z,
		raw.a_x, raw.a_y, raw.a_z);
	imuFilter.computeEuler();
  	double rotation[3][3];

	ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", deg_from_rad(imuFilter.getRoll()),
		deg_from_rad(imuFilter.getPitch()), deg_from_rad(imuFilter.getYaw()));
  	response.rpy.roll = deg_from_rad(imuFilter.getRoll());
  	response.rpy.pitch  = deg_from_rad(imuFilter.getPitch());
  	response.rpy.yaw = deg_from_rad(imuFilter.getYaw());
	imuFilter.getRotationMatrix(rotation);
	for(int i =0; i<3; i++)	{
		response.rot.row1[i] = rotation[0][i];
		response.rot.row2[i] = rotation[1][i];
		response.rot.row3[i] = rotation[2][i];
	}
	//Filling up pose header
//	response.pose.header.seq = seqNum;
	//response.pose.header.stamp = raw.timestamp;
	//response.pose.header.frame_id = 1;

	//Artificially setting to (0,0,0) position for testing
	//response.pose.pose.position.x = 0;
	//response.pose.pose.position.y = 0;
	//response.pose.pose.position.z = 0;


	imuFilter.getOrientation(	response.orientation.x,
								response.orientation.y,
								response.orientation.z,
								response.orientation.w);
  return true;
}


