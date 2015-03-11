/*
 * run_test_app.h
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#pragma once

#include "app.h"
#include <libsc/k60/lcd_typewriter.h>
#include <libutil/positional_pid_controller.h>

#define kp 0.9
#define ki	0
#define kd 	0
#define SETPOINT 12

namespace camera
{

class RunTestApp : public App
{
public:
	explicit RunTestApp(SystemRes *res)
				: App(res)
		{
		triggered = false;
		avg_width = 40;
		}

	void Run() override;
private:
	Byte* image = new Byte[80*60/8];
	std::vector<std::pair<int16_t, int16_t>> left;
	std::vector<std::pair<int16_t, int16_t>> right;
	bool triggered;
	int16_t avg_width;

	int16_t Analyze();
	bool Trigger(int32_t l_encoder_reading, int32_t r_encoder_reading);
	void FindMargin();
	bool is_error(int16_t row, int16_t position, bool side /*left = true, right=false*/);

};
}

