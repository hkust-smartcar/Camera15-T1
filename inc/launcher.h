/*
 * launcher.h
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#pragma once

#include "app.h"
#include "pid_controller.h"
#include "ImageProcess.h"
#include <libutil/sc_studio.h>

namespace camera
{

class Launcher : public App
{
public:
	explicit Launcher(SystemRes *res)
			: App(res),
			  l_m_setpoint(0),
			  r_m_setpoint(0),

			  l_kp(0.0108f),
			  l_ki(0.0f),
			  l_kd(0.00055f),

			  r_kp(0.0104f),
			  r_ki(0.0f),
			  r_kd(0.00055f),

			  multiple_PID(false),

			  l_speedControl(&l_m_setpoint, &l_kp, &l_ki, &l_kd,0,0,0, 0, 950),
			  r_speedControl(&r_m_setpoint, &r_kp, &r_ki, &r_kd,0,0,0, 0, 950),
			  imageProcess(GetSystemRes()->car)

	{
			data[0] = 1350.0f;
			data[1] = 0.46f;
			data[2] = 0.04f;

			data[3] = 0.4f;
			data[4] = 0.06f;

			l_m_setpoint = data[0];
			r_m_setpoint = data[0];

	}

	void Run();

private:
	void StartApp(const int id);
	void setParam(const int id);
	void setPID(const int id);

	float data[5];

	float l_m_setpoint;
	float r_m_setpoint;

	float l_kp;
	float l_ki;
	float l_kd;

	float r_kp;
	float r_ki;
	float r_kd;

	bool multiple_PID;

	PIDhandler l_speedControl;
	PIDhandler r_speedControl;

	ImageProcess  imageProcess;
};

}

/*
 	 	 	around 2m/s:
  			data[0] = 1200.0f;
			data[1] = 0.42f;
			data[2] = 0.0f;
			data[3] = 0.045f;

			faster?
  			data[0] = 1350.0f;
			data[1] = 0.46f;
			data[2] = 0.0f;
			data[3] = 0.046f;
*/
