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

#pragma once


#define HEIGHT 60
#define WIDTH 80

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
	double MidpointSumCal(Uint start, Uint end);
	void MedianFilter(bool* array_row, int length);

	~ImageProcess()
	{}

	bool bitmap[60][80];
	Uint margin[60][2];
	Uint midpoint[60];
	Uint MIDPOINT = 37;

	int16_t black_count = 0;
	int16_t white_count = 0;
	//for amplifying error
	Uint FACTOR = 100;

	//Q & cross road
	Uint start_row = 0;
	Uint end_row = HEIGHT;

	Uint black_end = 0;
	Uint white_start  = HEIGHT;
	Uint white_end = HEIGHT;

	//indicate situation
	bool Q = false;
	bool Qr = false;
	bool crossroad = false;
	bool l_byebye = false;
	bool r_byebye = false;
};
}


