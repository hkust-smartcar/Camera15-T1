/*
 * run_test_app.h
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#pragma once

#include "app.h"
#include "car.h"
#include "ImageProcess.h"
//#include "speedControl.h"
#include "MyVarManager.h"
#include <libsc/lcd_typewriter.h>
#include <libutil/positional_pid_controller.h>
#include "pid_controller.h"
#include <libsc/k60/jy_mcu_bt_106.h>

//#define kp 0.9
//#define ki	0
//#define kd 	0
//#define SETPOINT 12
#define SERVO_SETPOINT 0
//#define ENCODER_SETPOINT 50000

using namespace std;
using namespace libsc::k60;
using namespace libutil;

namespace camera
{

class RunTestApp : public App
{
public:
	explicit RunTestApp(SystemRes *res);
	void Run() override;

	RunTestApp &getInstance(void);

private:

	bool t = false;
	int setpower = 240;

	ImageProcess imageProcess;
//	speedControl speed;

	PositionalPidController<int,int> servoControl;
//	PositionalPidController<int,int> l_speedControl;
//	PositionalPidController<int,int> r_speedControl;

//	int encoder_count;
//	float skp;
//	float skd;

	float l_kp;
//	float* r_kp;

	float l_ki;
//	float* r_ki;

	float l_kd;
//	float r_kd;

	float reference;

	PIDhandler l_speedControl;

//	int16_t serror;
//	int16_t cal_result;
	int16_t ec1, ec2;
	int16_t l_result;
	int16_t r_result;
//	int16_t setpoint = ENCODER_SETPOINT/100;

	MyVarManager m_peter;

	static void PeggyListener(const std::vector<Byte> &bytes);

	void printMidpoint();
	void printMargin();


};
}
