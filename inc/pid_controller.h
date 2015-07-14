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

	explicit PIDhandler(float *ref, float *kp, float *ki, float *kd, const float min, const float max);
	float updatePID(float val);
	float updatePID_ori(float val);

	float getKp(void);
	float getKi(void);
	float getKd(void);

	void reset(void);

	float returnKpresult(){return *Kp * lastError;}
	float returnKiresult(){return *Ki * eSum;}
	float returnKdresult(){return *Kd * dE;}

private:

	float min;
	float max;

	float *reference;
	float *Kp;
	float *Ki;
	float *Kd;

	float dE;

	float eSum;
	float lastError;
	float epsilon;
	Timer::TimerInt lastTimeUpdate;

	float output;
};
