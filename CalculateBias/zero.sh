#!/bin/bash
clang++ zeroPhidget.cpp  spatial.cpp -framework Phidget21 -o calculateGyroBias; 
