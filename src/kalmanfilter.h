#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_


class KalmanFilter {

public :
KalmanFilter();
void configure(float zVariance, float zAccelVariance,  float zInitial, float vInitial);
void update(float z, float zAccelVariance,float dt, float* pZ, float* pV);

private :
// State being tracked
float z_;  // position
float v_;  // velocity

// 2x2 State Covariance matrix
float Pzz_;
float Pzv_;
float Pvz_;
float Pvv_;

float zAccelVariance_;  // dynamic acceleration variance
float zVariance_; //  z measurement noise variance fixed
};

#endif

