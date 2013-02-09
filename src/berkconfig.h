#ifndef BERKCONFIG
#define BERKCONFIG


#include "config.h"

//testmodes

#ifndef TESTNONE //allows for global disabling of test modes
  #ifndef BERKTESTER
  #define BERKTESTER //uncomment to activate testing mode for this file
  #endif //ifndef BERKTESTER
#endif // ifndef TTESTNONE
#ifdef TESTALL
  #ifndef BERKTESTER
  #define BERKTESTER //allows for testmode activation from config.h
  #endif //ifndef BERKTESTER
#endif //ifdef TESTALL

//end testmodes
//define unit conversions
#define deg_from_rad(X) ((X)* 57.2957795)
#define seconds_from_ms(x) ((x)/1000.0)

//define internal constants
#define gyroscopeErrorRate  0.01
// end  all constant definitions
//begin debugger code
  //make sure there is an \ at the end of every line where needed!
#define BERKTESTER0 #include <iostream> 	\
                    #include <fstream>

#define BERKTESTER1 											\
  fstream foutCurrentOr, foutRotMatrix; 						\
  foutCurrentOr.open("currentOrientation.txt", fstream::out); 	\
  foutcurrentOr << "Roll, Pitch, Yaw" << endl; 					\
  foutRotMatrix.open("rotMatrix.txt", fstream::out); 

#define BERKTESTER2 												\
		cout << "Rotation matrix" <<endl;							\
		for(int i =0; i< 3; i++)	{								\
			cout << "[ ";											\
			for(int k =0; k<3; k++)	{								\
				cout << rotMatrix[i][k] << " ";						\
			}														\
			cout << "]" << endl;									\
		}															\
																	\
		cout << "Updated Orientation" <<endl;						\
		cout << "Roll: " << orientation[0] << endl;					\
		cout << "Pitch: " << orientation[1] << endl;				\
		cout << "Yaw: " << orientation[2]<< endl;					\
		cout << "-------------------------------" << endl;

#endif
