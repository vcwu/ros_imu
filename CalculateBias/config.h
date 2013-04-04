
#ifndef CONFIG_H
#define CONFIG_H
#include <cmath>

//========================================
//DEBUGS
//========================================

#ifndef DEBUG
#define DEBUG  //remove this line when done debugging
#endif

#ifdef DEBUG

	#ifndef DEBUG_RAW_GYRO
	//#define DEBUG_RAW_GYRO
	#endif

	#ifndef DEBUG_ZERO_GYRO
	//#define DEBUG_ZERO_GYRO
	#endif

	#ifndef DEBUG_CURRENT_INTEGQUEUE
	#define DEBUG_CURRENT_INTEGQUEUE
	#endif
	
	#ifndef DEBUG_INTEGRATE
	//#define DEBUG_INTEGRATE
	#endif

	#ifndef DEBUG_FILTERING
	//#define DEBUG_FILTERING
	#endif

	//"@ Raw Phidget data, non zeroed, avg constant, then zeroed." 
	#ifndef DEBUG_LIVE_GRAPH_PHIDGET_RAW_GYRO
	#define DEBUG_LIVE_GRAPH_PHIDGET_RAW_GYRO
	#endif

	#ifndef DEBUG_LIVE_GRAPH_PHIDGET_RAW_ACC	
	#define DEBUG_LIVE_GRAPH_PHIDGET_RAW_ACC
	#endif


	//"@ Rotated gyro data, after zeroing
	#ifndef DEBUG_LIVE_GRAPH_ROTATION
	#define DEBUG_LIVE_GRAPH_ROTATION 
	#endif

	//
	#ifndef DEBUG_LIVE_GRAPH_DELTA
	#define DEBUG_LIVE_GRAPH_DELTA
	#endif

	//"@ Current Orientation, rotated, then non filtered and filtered."
	#ifndef DEBUG_LIVE_GRAPH_CURRENT_ORIENTATION
	#define DEBUG_LIVE_GRAPH_CURRENT_ORIENTATION
	#endif


	#ifndef DEBUG_ROTATION_MATRIX
	#define DEBUG_ROTATION_MATRIX
	#endif


//DEBUG
#endif	


//========================================
//CONSTANTS

//========================================

const double US_PER_SECOND = 1000000;
const int dataRate = 16;
const double TAU = 50;
const double RAD =  std::atan(1)/45; //(pi/4)/45 = pi/180 radians per degree
const double GYRO_OFFSET[3] = {-0.241803, 0.02563817, -0.3029};
const double ACC_OFFSET[3] = {0.0203066, -0.0137503, 1.000767};
//CONFIG_H
#endif
