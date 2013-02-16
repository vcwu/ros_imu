#include "ros/ros.h"
#include "std_msgs/String.h"
#include <IMU/spatialRaw.h>
#include "phidget_headers/spatial_helper.h"
#include <IMU/orientation.h>
#include <IMU/imu_filter.h>
#include "config.h"
extern pthread_mutex_t mutex; 	//used when handling data q
extern pthread_cond_t cond;		//used when handling data  q
using namespace std;

void copySpatialRaw(spatial::PhidgetRawDataQ::iterator it, IMU::spatialRaw *msg)	{

	//Copying Timestamp
	msg->timestamp.sec = it->timestamp.seconds;
	msg->timestamp.nsec = it->timestamp.microseconds;

	//Copying Acceleration (in g's)
	msg->a_x = it->acceleration[0];
	msg->a_y = it->acceleration[1];
	msg->a_z = it->acceleration[2];

	//Copying Angular Rate (in deg/s)
	msg->w_x = it->angularRate[0];
	msg->w_y = it->angularRate[1];
	msg->w_z = it->angularRate[2];

	//Copying Magnetic field (in gauss)
	msg->m_x = it->magneticField[0];
	msg->m_y = it->magneticField[1];
	msg->m_z = it->magneticField[2];

}

void fillSpatialMsg(spatial::PhidgetRawDataQ::iterator it, spatial::PhidgetRawDataQ* dataQueue, IMU::spatialRaw *msg)	{
	ROS_INFO("Filling up msg");
	//Dealing with data Queue
	pthread_mutex_lock(&mutex);

	//Check to see if dataQueue is empty. 
	//If it is, wait.
	if(dataQueue->empty())	{
		pthread_cond_wait(&cond, &mutex);
	}
	ROS_INFO("Not empty!"); 
	
	it = dataQueue->begin();
	copySpatialRaw(it, msg);
	dataQueue->pop_front();
	pthread_mutex_unlock(&mutex);


}

int main(int argc, char* argv[]){

	//ROS Setup
	//-------------------------------
	ROS_INFO("Initializing IMU talker");
  	int ROSbufferSize = 10, ROScount = 0;
 	ros::init(argc, argv, "Phidget_Stuff");
  	ros::NodeHandle PhidgetNode;
  	ros::Rate loop_rate(10);
  	ros::Publisher PhidgetPub = 
    	PhidgetNode.advertise<IMU::spatialRaw>("IMU_data", ROSbufferSize);
	ros::Publisher OrientationPub = PhidgetNode.advertise<IMU::orientation>("Orientation_data", ROSbufferSize);
	ros::Publisher RotMatrixPub = PhidgetNode.advertise<IMU::rotMatrix>("Rotation_Matrix", ROSbufferSize);
	ros::ServiceClient client = PhidgetNode.serviceClient<IMU::imu_filter>("Calculate_Orientation");
	
	//Creating/Initializing Spatial Handle
	//------------------------------------
	ROS_INFO("Initializing Spatial handles");
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

	//init cond
	//-----------------------------------
	if(pthread_cond_init(&cond, NULL)!=0)	{
		ROS_INFO("cond init failed!");
	}
	else	{
		ROS_INFO("cond init success \n");
	}
	
	//set up spatial
	//-------------------------------------
	spatial::spatial_setup(spatial, dataQueue, DATA_RATE);

 	while(ros::ok()) {
		

		//Publishing raw IMU data
		IMU::spatialRaw  msg;
			
		fillSpatialMsg(it, dataQueue, &msg);

	//	ROS_INFO("Time %ds %dns", msg.timestamp.sec, msg.timestamp.nsec);
	//	ROS_INFO("Gyr X:%f Y:%f Z:%f", msg.w_x, msg.w_y, msg.w_z);	
		PhidgetPub.publish(msg);
	
		//Publishing orientation data
		
		IMU::imu_filter srv;
		srv.request.rawIMU = msg;
		if(client.call(srv))	{
			ROS_INFO("Orientation_Calculate call successful.");
			ROS_INFO("Roll: %f", srv.response.orientation.roll);
			ROS_INFO("pitch: %f", srv.response.orientation.pitch);
			ROS_INFO("yaw: %f", srv.response.orientation.yaw);
			OrientationPub.publish(srv.response.orientation);
			RotMatrixPub.publish(srv.response.rot);
			
		}
		else	{
			ROS_ERROR("Failed to call service Calculate_orientation");
		}
	
    	ROScount++;
//		loop_rate.sleep();
    	ros::spinOnce();
 	}
}
