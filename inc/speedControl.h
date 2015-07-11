///*
// * speedControl.h
// *
// * Author: Ben Lai, Ming Tsang, Peggy Lau
// * Copyright (c) 2014-2015 HKUST SmartCar Team
// * Refer to LICENSE for details
// */
//
//#include "libutil/misc.h"
//#include "car.h"
//#include <libsc/system.h>
//#include <algorithm>
//
//#pragma once
//using namespace libsc::k60;
//using namespace libsc;
//
//namespace camera
//{
//
//class speedControl{
//
//public:
//	speedControl(float kp, float ki, float kd);
//	~speedControl();
//
//	int speedCal(Car* carpointer, int power);
//
//private:
//	int32_t m_prev_error;
//	int32_t m_prev_degree;
//	Timer::TimerInt prev_time;
//
//	float m_kp;
//	float m_ki;
//	float m_kd;
//
//};
//
//}
