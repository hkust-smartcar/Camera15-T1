/*
 * PIDhandler.h
 *
 *  Created on: Jan 15, 2015
 *      Author: Peter
 */

#include <libsc/system.h>

#pragma once

#define MAX(a, b) ((a > b)? a : b)

#define EPSILON_RATIO			0.05

using namespace libsc;

class PIDhandler
{
public:

	explicit PIDhandler(float *ref, float *kp, float *ki, float *kd, float *kp_straight,float *ki_straight,float *kd_straight,const float min, const float max);
	float updatePID(float val);
	float updatePID_ori(float val);

	float getKp(void);
	float getKi(void);
	float getKd(void);

	void reset(void);

	float returnKpresult(){return *Kp * lastError;}
	float returnKiresult(){return *Ki * eSum;}
	float returnKdresult(){return *Kd * dE;}

	int8_t returnTypeOfPID(){return TypeOfPID;}

private:

	float min;
	float max;

	float *reference;
	float *Kp;
	float *Ki;
	float *Kd;

	float *Kp_straight;
	float *Ki_straight;
	float *Kd_straight;

	float dE;

	float eSum;
	float lastError;
	float epsilon;
	Timer::TimerInt lastTimeUpdate;

	float output;

	int8_t TypeOfPID;
};
