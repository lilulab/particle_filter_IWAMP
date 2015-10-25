#ifndef __CONSTANTS_H_
#define __CONSTANTS_H_

#include <cmath>

// Sizes
#define STATE_SIZE 3 //state: x,y,theta
#define INPUT_SIZE 2 //input: encoder_left, encoder_right
#define MEAS_SIZE 1  //measurement: using similarity score as measurement

#define NUM_SAMPLES 500 // Default Number of Samples
#define RESAMPLE_PERIOD 0 // Default Resample Period
#define RESAMPLE_THRESHOLD (NUM_SAMPLES/4.0) // Threshold for Dynamic Resampling

// Prior:
// Initial estimate of position and orientation
#define PRIOR_MU_X 0.1
#define PRIOR_MU_Y -0.4
#define PRIOR_MU_THETA 0	//M_PI/4

// Initial covariances of position and orientation
#define PRIOR_COV_X pow(0.05,2)
#define PRIOR_COV_Y pow(0.05,2)
#define PRIOR_COV_THETA pow(0.05*M_PI/180,2)

// System Noise
#define MU_SYSTEM_NOISE_X 0.0 
#define MU_SYSTEM_NOISE_Y 0.0 
#define MU_SYSTEM_NOISE_THETA 0.0
#define SIGMA_SYSTEM_NOISE_X pow(0.05,2)
#define SIGMA_SYSTEM_NOISE_Y pow(0.05,2)
#define SIGMA_SYSTEM_NOISE_THETA pow(0.05*M_PI/180,2)

// Measurement noise
#define SIGMA_MEAS_NOISE pow(2,2) // If this sigma is too small, the calculated probability can potentially be zero!!!
#define MU_MEAS_NOISE 0.0

// Wingbay Boundary
#define X_MAX  0.3
#define X_MIN -0.3
#define Y_MAX -0.16
#define Y_MIN -1.24

#endif
