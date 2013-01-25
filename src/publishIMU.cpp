#include "ros/ros.h"
#include "std_msgs/String.h"
#include <IMU/spatialRaw.h>
#include "spatial_helper.h"
#include "config.h"
extern pthread_mutex_t mutex; 	//used when handling data q

using namespace std;

int main(int argc, char* argv[]){

  	int ROSbufferSize = 100, ROScount = 0;
 	ros::init(argc, argv, "Phidget_Stuff");
  	ros::NodeHandle PhidgetNode;
  	ros::Publisher PhidgetPub = 
    	PhidgetNode.advertise<IMU::spatialRaw>("IMU_data", ROSbufferSize);
  	ros::Rate loop_rate(1);
	
	//Creating/Initializing Spatial Handle
	//------------------------------------
	CPhidgetSpatialHandle spatial =0;
	CPhidgetSpatial_create(&spatial);

	//Setting up data q
	//------------------------------------
	spatial::PhidgetRawDataQ* dataQueue = new spatial::PhidgetRawDataQ();
	spatial::PhidgetRawDataQ::iterator it = dataQueue->begin();
	
	//init mutex
	//-------------------------------------
	if(pthread_mutex_init(&mutex, NULL)!= 0)	{
		ROS_INFO("mutex init failed");
		return -1;	//erm this is bad
	}
	else	{
		ROS_INFO("mutex init success \n");
	}

	//set up spatial
	//-------------------------------------
	spatial::spatial_setup(spatial, dataQueue, DATA_RATE);


  	while(ros::ok()) {
		
		//Getting data from phidget
		while(dataQueue->empty())	{}	//spinlock - bad ? :(
		
		pthread_mutex_lock(&mutex);
		it = dataQueue->begin();
		//do more stuff im tired
		pthread_mutex_unlock(&mutex);
		//Filling up 
		IMU::spatialRaw  msg;
			

		ROS_INFO("Time %ds %dns", msg.timestamp.sec, msg.timestamp.nsec);
	
//		PhidgetPub.publish(msg);
	

    	ROScount++;
		loop_rate.sleep();
    	ros::spinOnce();
 	}
}
