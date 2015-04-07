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
#include "speedControl.h"
#include <libsc/lcd_typewriter.h>
#include <libutil/positional_pid_controller.h>
#include <libsc/k60/jy_mcu_bt_106.h>

#define kp 0.9
#define ki	0
#define kd 	0
#define SETPOINT 12

using namespace std;
using namespace libsc::k60;


namespace camera
{

class RunTestApp : public App
{
public:
	explicit RunTestApp(SystemRes *res);

	void Run() override;
private:
	int16_t black_count;

	bool t = false;
	int setpower = 240;

	ImageProcess imageProcess;

	int FACTOR = 100;

	int Analyze(void);
	double MidpointSumCal(int start, int end);

	void printMidpoint();
	void printMargin();


};
}
