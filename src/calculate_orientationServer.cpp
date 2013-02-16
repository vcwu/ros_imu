/******************************************************************************
* Calculate orientation SERVER
* the orientation service takes raw IMU data and converts it to a useable
*   odometry message
*	
* IMU filter 
* addtl notes: if you're going to be modifying this code, be sure to familarize 
*   yourself with the constants defined in berkconfig.h
*
******************************************************************************/

#include "ros/ros.h"
//Message Types
//#include <IMU/spatialRaw.h>
//#include <IMU/orientation.h>
#include <IMU/imu_filter.h>
//Our headers
#include "orientation_headers/imuFilter.h"
#include "config.h" 

IMUfilter imuFilter(seconds_from_ms(DATA_RATE), gyroscopeErrorRate);

bool calculate(IMU::imu_filter::Request &request, IMU::imu_filter::Response &response);

int main(int argc, char* argv[])
{
	
	ros::init(argc, argv, "Calculate_Orientation_server");
	ros::NodeHandle berk;
	
	imuFilter.reset();
	ros::ServiceServer service = berk.advertiseService("Calculate_Orientation", calculate );
	ROS_INFO("Ready to calculate orientation.");
	ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", imuFilter.getRoll(),
		imuFilter.getPitch(), imuFilter.getYaw());
	ros::spin();
	return 0;
}



bool calculate(IMU::imu_filter::Request &request, IMU::imu_filter::Response &response)
{
	//Update filter with IMU data
	IMU::spatialRaw raw = request.rawIMU;
	ROS_INFO("Server Side, Raw Phidget Data");
	ROS_INFO("a_x: %f, a_y: %f, a_z:%f", raw.a_x, raw.a_y, raw.a_z);
	ROS_INFO("w_x: %f, w_y: %f, w_z:%f", raw.w_x, raw.w_y, raw.w_z);
	
	imuFilter.updateFilter(raw.w_x, raw.w_y, raw.w_z,
		raw.a_x, raw.a_y, raw.a_z);
	imuFilter.computeEuler();
  	double rotation[3][3];

	ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", imuFilter.getRoll(),
		imuFilter.getPitch(), imuFilter.getYaw());
  	response.orientation.roll = deg_from_rad(imuFilter.getRoll());
  	response.orientation.pitch  = deg_from_rad(imuFilter.getPitch());
  	response.orientation.yaw = deg_from_rad(imuFilter.getYaw());
	imuFilter.getRotationMatrix(rotation);
	for(int i =0; i<3; i++)	{
		response.rot.row1[i] = rotation[0][i];
		response.rot.row2[i] = rotation[1][i];
		response.rot.row3[i] = rotation[2][i];
	}
	//Artificially setting to (0,0,0) position for testing
	response.pose.position.x = 0;
	response.pose.position.y = 0;
	response.pose.position.z = 0;

	imuFilter.getOrientation(	response.pose.orientation.x,
								response.pose.orientation.y,
								response.pose.orientation.z,
								response.pose.orientation.w);
  return true;
}


