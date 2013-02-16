#ifndef CONFIG_H
#define CONFIG_H


#define gyroscopeErrorRate  0.01
#define deg_from_rad(X) ((X)* 57.2957795)
#define seconds_from_ms(x) ((x)/1000.0)
#define DATA_RATE 16	//phidget pushes packets every 16 millisec
const double GYRO_OFFSET[3] = {-0.241803, 0.02563817, -0.3029};
/*SPATIAL_CONFIG_H*/
#endif
