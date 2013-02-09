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

bool calculate(IMU::imu_filter::Request &request, IMU::imu_filter::Response &response)
{
  //this is where the berk code goes
  //Raw IMU data is request.rawIMU.*
  // output is response.* <--these may stil need to be set in the srv file

/*
  double orientation[3], rotMatrix[3][3]; 
  response.roll = deg_from_rad(imuFilter.getRoll());
  response.pitch  = deg_from_rad(imuFilter.getPitch());
  response.yaw = deg_from_rad(imuFilter.getYaw());
*/
//  imufilter.getRotationMatrix(response.rotMatrix);

#ifdef BERKTESTER
//BERKTESTER2//print out any statements
#endif

  return true;
}

int main(int argc, char* argv[])
{

#ifdef BERKTESTER
//BERKTESTER1 //define filestreams
#endif //ifdef BERKTESTER

	IMUfilter imuFilter(seconds_from_ms(DATA_RATE), gyroscopeErrorRate);
	
	ros::init(argc, argv, "Calculate_Orientation_server");
	ros::NodeHandle berk;

	ros::ServiceServer service = berk.advertiseService("Calculate_Orientation", calculate);
	ROS_INFO("Ready to calculate orientation.");
	ros::spin();
	return 0;
}
