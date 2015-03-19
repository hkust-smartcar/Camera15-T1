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
#include <libsc/k60/lcd_typewriter.h>
#include <libutil/positional_pid_controller.h>

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
	explicit RunTestApp(SystemRes *res)
	: App(res)
	{
		triggered = false;
		RightAngle = false;
		initiate = false;
		black_count = 0;
		avg_width = 40;
	}

	void Run() override;
private:
	bool triggered;
	bool RightAngle;
	int16_t black_count;
	int16_t avg_width;
	bool initiate;

	bool Trigger(int32_t l_encoder_reading, int32_t r_encoder_reading);
	bool is_error(int16_t image_row,int16_t position,bool prev);


	int16_t Analyze();
	vector<pair<int16_t, int16_t>> midpoint();
	void FindMargin(Byte* image, LcdTypewriter writer);
	double AvgCal(vector<pair<int16_t, int16_t>>midpoint, int start, int end);

};
}
