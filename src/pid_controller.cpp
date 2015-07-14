/*
 * PIDhandler.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: Peter
 */

#include <cstring>

#include <libsc/system.h>

#include "pid_controller.h"

#include <libutil/misc.h>

using namespace libsc;

#define abs(v) ((v > 0)? v : -v)
#define	inRange(n, v, x) ((v > x)? x : ((v < n)? n : v))

PIDhandler::PIDhandler(float *ref, float *kp, float *ki, float *kd, const float min, const float max)
:
	reference(ref),
	Kp(kp),
	Ki(ki),
	Kd(kd),
	eSum(0),
	output(0),
	lastError(0),
	epsilon(*reference * EPSILON_RATIO),
	lastTimeUpdate(0),
	min(min),
	max(max)
{
	System::Init();
	reset();
}

float PIDhandler::getKp(void)
{
	return *Kp;
}

float PIDhandler::getKi(void)
{
	return *Ki;
}

float PIDhandler::getKd(void)
{
	return *Kd;
}

void PIDhandler::reset(void)
{
	eSum = 0;
	lastTimeUpdate = System::Time();
	output = 0;

}

float PIDhandler::updatePID(float val)
{
	float error = *reference - val;
	float dt = Timer::TimeDiff(System::Time(), lastTimeUpdate)/1000.0f;
//	float
	dE = (error - lastError) / dt;
	lastError = error;

	if (abs(lastError) >= epsilon)
		eSum += error*dt;

	float m_i_limit = 100;
	float I = *Ki*eSum;

	I = libutil::Clamp<float>(-m_i_limit, I, m_i_limit);

	output += *Kp * lastError + I + *Kd * dE;
	lastTimeUpdate = System::Time();

	return inRange(min, output, max);

}

float PIDhandler::updatePID_ori(float val)
{
	float error = *reference - val;
	uint32_t dt = Timer::TimeDiff(System::Time(), lastTimeUpdate);
	float dE = (error - lastError) / dt;
	lastError = error;


	output = *Kp * lastError/* - *Kp * lastError * abs(lastError)*/ + *Ki * eSum + *Kd * dE;

	lastTimeUpdate = System::Time();

	return inRange(min, output, max);

}

