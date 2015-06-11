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

#include "distortion.h"
#pragma once


#define HEIGHT 60
#define WIDTH 80

#define INIT_STATE 	 0;
#define STRAIGHT 	 1;
#define CROSSROAD 	 2;
#define RIGHT_ANGLE  3;
#define Q_TURN 		 4;
#define TURNING 	 5;
#define OUT_OF_BOUND 6;

using namespace libsc::k60;
using namespace libsc;
using namespace std;


namespace camera
{
class ImageProcess{
public:
	ImageProcess();
	void start(Byte* image);

	int Analyze(void);
	double MidpointSumCal(uint16_t start, uint16_t end);
	~ImageProcess()
	{}

	bool bitmap[60][80];
	uint16_t margin[60][2];
	uint16_t midpoint[60];
	uint16_t MIDPOINT = 37;

	int16_t black_count = 0;
	int16_t white_count = 0;
	//for amplifying error
	uint16_t FACTOR = 100;

	//Q & cross road
	uint16_t start_row = 0;
	uint16_t end_row = HEIGHT;

	uint16_t black_end = 0;
	uint16_t checkRA = 0;
	uint16_t white_start  = HEIGHT;
	uint16_t white_end = HEIGHT;
	uint16_t black_line_start = 0;
	uint16_t black_line_end = 0;

	int16_t data[60][4]; //[0]: number of white pixels; [1]: white row; [2]: black row; [3]: more white at left(0)/right(1)


	//indicate situation
	bool Qr = false;
	bool crossroad = false;
	bool l_byebye = false;
	bool r_byebye = false;
	bool right_angle = false;
	bool black_line = true;

	//State
	int STATE = 0;

	//distortion
	distortion dis;


};
}


