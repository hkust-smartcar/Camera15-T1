/*
 * speedControl.cpp
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include "speedControl.h"
#include <stdlib.h>

namespace camera
{
speedControl::speedControl(float kp, float ki, float kd)
{
	m_kp = kp;
	m_ki = ki;
	m_kd = kd;

	m_prev_error = 0;
	m_prev_degree = 950;
	prev_time = System::Time();
}

speedControl::~speedControl()
{}

int speedControl::speedCal(Car* carpointer){

	if (abs(carpointer->GetServo().GetDegree()-950)<8){
		m_prev_degree = carpointer->GetServo().GetDegree();
		if (carpointer->GetMotor(0).GetPower()+10 < 1000)
			return carpointer->GetMotor(0).GetPower()+10;
		else
			return carpointer->GetMotor(0).GetPower();
	}
	else if(carpointer->GetServo().GetDegree()>1100){
		return 180;
	}
	else{
		m_prev_degree = carpointer->GetServo().GetDegree();
		return 300;
	}
}

}