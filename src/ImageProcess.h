/*
 * ImageProcess.h
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */
#include "libutil/misc.h"
#include <libsc/system.h>
#include <algorithm>
#include "libsc/k60/jy_mcu_bt_106.h"
#include "blacklineProcess.h"
#include "VerticalImageProcess.h"
#include "Imp.h"
#include "car.h"

#include "definition.h"

//#include "distortion.h"
#pragma once

using namespace libsc::k60;
using namespace libsc;
using namespace std;
using namespace libutil;


namespace camera
{
class ImageProcess{

public:

	ImageProcess(Car* car_ptr);
	void start(Byte* image);

	float Analyze(void);
	float MidpointSumCal(uint16_t start, uint16_t end);
	~ImageProcess()
	{}
	void printResult();
	int getState(){return STATE;}

private:

	Car* car;

	bool bitmap[58][78];
	uint8_t margin[58][2];
	uint8_t midpoint[58];
	uint8_t bias_crossroad_margin[58][4];

	uint8_t black_count;
	uint8_t white_count;
	//for amplifying error
	uint8_t FACTOR;

	//black line & cross road

	uint8_t black_end;
	uint8_t checkRA;
	uint8_t white_start;
	uint8_t white_end;
	uint8_t black_line_start;
	uint8_t black_line_end;
	uint8_t double_check_black_end;

	Timer::TimerInt RA_time;
	Timer::TimerInt BL_time;

	float left_slope;
	float right_slope;
	float slope;

	uint16_t data[58][4]; //[0]: number of white pixels; [1]: white row; [2]: black row; [3]: more white at left(0)/right(1)

	//indicate situation
	bool crossroad;
	bool l_byebye;
	bool r_byebye;
	bool right_angle;
	bool black_line;
	bool bg;
//	bool continue_right_angle;
	bool bias_crossroad;
	bool continue_bias_crossroad;
	bool first_black_line;

	//black guide line
	blacklineProcess blp;
	uint16_t almost_white_number;

	//vertical scaning
	VerticalImageProcess vip;

	//State
	uint8_t STATE;
	float prev_error;

	//distortion
	//	distortion dis;

	//median filter
	Imp medianFilter;


};
}


