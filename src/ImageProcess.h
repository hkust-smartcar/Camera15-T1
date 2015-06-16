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

#include "distortion.h"
#pragma once


#define HEIGHT 60
#define WIDTH 80

#define INIT_STATE 	 0
#define STRAIGHT 	 1
#define CROSSROAD 	 2
#define RIGHT_ANGLE  3
#define Q_TURN 		 4
#define TURNING 	 5
#define OUT_OF_BOUND 6
#define MIDPOINT_REF 37

using namespace libsc::k60;
using namespace libsc;
using namespace std;
using namespace libutil;


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

	int16_t black_count;
	int16_t white_count;
	//for amplifying error
	uint16_t FACTOR;

	//black line & cross road

	uint16_t black_end;
	uint16_t checkRA;
	uint16_t white_start;
	uint16_t white_end;
	uint16_t black_line_start;
	uint16_t black_line_end;

	float slope;

	int16_t data[60][4]; //[0]: number of white pixels; [1]: white row; [2]: black row; [3]: more white at left(0)/right(1)


	//indicate situation
	bool crossroad;
	bool l_byebye;
	bool r_byebye;
	bool right_angle;
	bool black_line;
	bool bg;

	//black guide line
	blacklineProcess blp;
	uint16_t almost_white_number;

	//State
	int STATE;

	//distortion
	distortion dis;


};
}


