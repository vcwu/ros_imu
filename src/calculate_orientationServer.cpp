/******************************************************************************
* orientation SERVER
* the orientation service takes raw IMU data and converts it to a useable
*   odometry message
*
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
#include "berkconfig.h" //for error rates, other constants ->should include config.h

/*
bool callback(std_servs::Empty::Request&, std_srvs::Empty::Response& response)	{
	return true;
}*/

#ifdef BERKTESTER
//BERKTESTER0 //includes needed for other BERKTESTSER code
#endif


IMUfilter imuFilter(seconds_from_ms(DATA_RATE), gyroscopeErrorRate);

bool calculate(IMU::imu_filter::Request &request, IMU::imu_filter::Response &response);

int main(int argc, char* argv[])
{

#ifdef BERKTESTER
//BERKTESTER1 //define filestreams
#endif //ifdef BERKTESTER

	
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
	ROS_INFO("a_x: %f, a_y: %f, a_z:%f", raw.a_x, raw.a_y, raw.a_z);
	ROS_INFO("w_x: %f, w_y: %f, w_z:%f", raw.w_x, raw.w_y, raw.w_z);
	
	imuFilter.updateFilter(raw.w_x, raw.w_y, raw.w_z,
		raw.a_x, raw.a_y, raw.a_z);
	imuFilter.computeEuler();
  	double rotMatrix[3][3];

	ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", imuFilter.getRoll(),
		imuFilter.getPitch(), imuFilter.getYaw());
  	response.roll = deg_from_rad(imuFilter.getRoll());
  	response.pitch  = deg_from_rad(imuFilter.getPitch());
  	response.yaw = deg_from_rad(imuFilter.getYaw());
//  imufilter.getRotationMatrix(response.rotMatrix);

#ifdef BERKTESTER
//BERKTESTER2//print out any statements
#endif
	  return true;
}


