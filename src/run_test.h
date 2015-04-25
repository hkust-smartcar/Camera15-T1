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
#include "MyVarManager.h"
#include <libsc/lcd_typewriter.h>
#include <libutil/positional_pid_controller.h>
#include "pid_controller.h"
#include <libsc/k60/jy_mcu_bt_106.h>
#include "motor_sd.h"

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

	//kp, ki, kd and setpoint for servo
	float s_kp;
	float s_ki;
	float s_kd;

	float s_setpoint;

	//kp, ki, kd and setpoint for motor
	float l_kp;
	float r_kp;

	float l_ki;
	float r_ki;

	float l_kd;
	float r_kd;

	float l_m_setpoint;
	float r_m_setpoint;

	// for software differential
	int16_t sd_setpoint;

	//servo_pid
	int16_t s_degree;
	int16_t s_result;

	//motor_pid
	int16_t ec0, ec1;
	int16_t l_result;
	int16_t r_result;


	//pid controller for servo & motor
	PIDhandler servo_Control;
	PIDhandler l_speedControl;
	PIDhandler r_speedControl;

	//software differential
	SD software_differential;
	MyVarManager m_peter;
	ImageProcess imageProcess;

	static void PeggyListener(const std::vector<Byte> &bytes);


};
}
