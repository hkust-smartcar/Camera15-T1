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
#include "definition.h"

using namespace std;
using namespace libsc::k60;


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
	float speed = 0;

	//kp, ki, kd and setpoint for servo
	float s_kp;
	float s_ki;
	float s_kd;

	float s_setpoint;

	//kp, ki, kd and setpoint for motor
	float l_kp;
	float l_ki;
	float l_kd;

	float r_kp;
	float r_ki;
	float r_kd;

	float l_m_setpoint;
	float r_m_setpoint;

	// for software differential
	int16_t sd_setpoint;

	//servo_pid
	int16_t s_degree;
	int16_t s_result;
	float show_error;

	//motor_pid
	int16_t ec0, ec1;
	int32_t l_result;
	int32_t r_result;
	int16_t sp_storage[2];


	//pid controller for servo & motor
	PIDhandler servo_Control;
	PIDhandler l_speedControl;
	PIDhandler r_speedControl;

	//software differential
	SD software_differential;
	MyVarManager m_peter;
	ImageProcess imageProcess;

	struct EmergencyStopState
		{
			libsc::Timer::TimerInt trigger_time = 0;
			bool is_triggered = false;
		};

	libsc::Timer::TimerInt m_start;
	bool m_is_stop;
	EmergencyStopState m_emergency_stop_state;

	static void PeggyListener(const std::vector<Byte> &bytes);
	void printResult();
//	void updateSPD(float error);
	void peggy(libbase::k60::Pit* pit);
	void DetectEmergencyStop();


};
}
