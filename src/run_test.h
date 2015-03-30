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
	  bt(bt_config())
	{
		RightAngle = false;
		CrossRoad = false;
		initiate = false;
		black_count = 0;
		avg_width = 40;
	}

	void Run() override;
private:
	bool RightAngle;
	bool CrossRoad;
	int16_t black_count;
	int16_t avg_width;
	bool initiate;
	int FACTOR = 170;

	int margin[60][2];
	int midpoint[60];

	bool Trigger(int32_t l_encoder_reading, int32_t r_encoder_reading);
	bool is_error(int left, int right);

	int Analyze(void);
	void cal_midpoint(void);
	void FindMargin(Byte* image);
	double AvgCal(int start, int end);

	void printMidpoint();
	void printMargin();
	JyMcuBt106 bt;

	JyMcuBt106::Config bt_config(){
		JyMcuBt106::Config config;
		config.id = 0;
		config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
		config.rx_irq_threshold = 1;
		config.rx_isr = true;
		return config;
	}



};
}
