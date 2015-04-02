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
	explicit RunTestApp(SystemRes *res)
	: App(res),
	  speed(0,0,0)
	{
		RightAngle = false;
//		CrossRoad = false;
//		initiate = false;
		black_count = 0;
//		avg_width = 40;

		for(int i=0; i<60; i++){
			midpoint[i] = 37;
		}

	}

	void Run() override;
private:
	bool RightAngle;
	int16_t black_count;
	int white_after_black = 0;
	int out_indicator = 0;

	speedControl speed;

	int FACTOR = 100;

	int margin[60][2];
	int midpoint[60];

	int MIDPOINT = 35;
			//37;

	bool Trigger(int32_t l_encoder_reading, int32_t r_encoder_reading);
	int Analyze(void);
	//void cal_midpoint(void);
	void FindMargin(Byte* image);
	double MidpointSumCal(int start, int end);

	void printMidpoint();
	void printMargin();

	void MedianFilter(bool* array_row, int length);
	bool check_going_out();

	//bool CrossRoad;
	//int16_t avg_width;
	//	bool initiate;

};
}
