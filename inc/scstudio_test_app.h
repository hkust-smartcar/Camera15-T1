/*
 * scstudio_test_app.h
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#pragma once

#include <libsc/lcd_typewriter.h>
#include <libutil/positional_pid_controller.h>
#include <libutil/sc_studio.h>

#include "app.h"
#include "ImageProcess.h"
#include "MyVarManager.h"
#include "pid_controller.h"
#include "motor_sd.h"
#include "definition.h"

using namespace std;
using namespace libsc::k60;


namespace camera
{

class SCStudioTestApp : public App
{
public:
	explicit SCStudioTestApp(SystemRes *res, uint16_t motor_setpoint, float skp, float ski, float skd, float s_skp, float s_skd);
	void Run() override;

private:

	//indicate if motor is running
	bool motor_run = true;

	//kp, ki, kd and setpoint for servo
	float s_kp;
	float s_ki;
	float s_kd;

	float s_kp_straight;
	float s_ki_straight;
	float s_kd_straight;

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
	uint16_t sd_setpoint;

	//servo_pid
	float show_error;

	//motor_pid
	int16_t ec0, ec1;

	//hold right angle
	Timer::TimerInt RA_time;

	//pid controller for servo & motor
	PIDhandler servo_Control;
	PIDhandler l_speedControl;
	PIDhandler r_speedControl;

	//software differential
	SD software_differential;

	//ImageProcess
	ImageProcess imageProcess;

	//Safety
	struct EmergencyStopState
		{
			libsc::Timer::TimerInt trigger_time = 0;
			bool is_triggered = false;
		};

	libsc::Timer::TimerInt m_start;
	bool m_is_stop;
	EmergencyStopState m_emergency_stop_state;
	void DetectEmergencyStop();

	ScStudio scstudio;

	//watch adc and gpo data
	float prev_adc;
	float gpo;


};
}
