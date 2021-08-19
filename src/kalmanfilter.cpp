#include <Arduino.h>
#include <math.h>
#include "kalmanfilter.h"


KalmanFilter::KalmanFilter(){}


void KalmanFilter::configure(float zVariance, float zAccelVariance, float zInitial, float vInitial) {
	zAccelVariance_ = zAccelVariance;
	zVariance_ = zVariance;
	z_ = zInitial;
	v_ = vInitial;
	Pzz_ = 10.0f;
	Pzv_ = 0.0f;
	Pvz_ = 0.0f;
	Pvv_ = zAccelVariance_;
	}


// Updates state given a sensor measurement of z, the acceleration noise variance,
// and the time in seconds since the last measurement.
// This interval must be greater than 0; for the first measurement after a reset(),
// it's safe to use 1.0.

void KalmanFilter::update(float z, float zAccelVariance, float dt, float* pZ, float* pV) {
	zAccelVariance_ = zAccelVariance;

	// Update state estimate.
	z_ += v_ * dt;

	// Update state covariance. The last term mixes in acceleration noise.
	Pzz_ += dt*Pzv_ + dt*Pvz_ + dt*dt*Pvv_ + zAccelVariance_*dt*dt*dt*dt/4.0f;
	Pzv_ +=                        dt*Pvv_ + zAccelVariance_*dt*dt*dt/2.0f;
	Pvz_ +=                        dt*Pvv_ + zAccelVariance_*dt*dt*dt/2.0f;
	Pvv_ +=                                + zAccelVariance_*dt*dt;

	// Update step.
	float y = z - z_;  // Innovation.
	float sInv = 1.0f / (Pzz_ + zVariance_);  // Innovation precision.
	float kz = Pzz_ * sInv;  // Kalman gain
	float kv = Pzv_ * sInv;

	// Update state estimate.
	z_ += kz * y;
	v_ += kv * y;

	*pZ = z_;
	*pV = v_;

	// Update state covariance.
	Pvv_ -= Pzv_ * kv;
	Pzv_ -= Pzv_ * kz;
	Pvz_ -= Pzz_ * kv;
	Pzz_ -= Pzz_ * kz;
	}
