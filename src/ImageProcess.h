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
	double MidpointSumCal(int start, int end);

	~ImageProcess()
	{}

	bool bitmap[60][80];
	int margin[60][2];
	int midpoint[60];
	int MIDPOINT = 37;

	int16_t black_count = 0;
	int16_t white_count = 0;
	int FACTOR = 90;

	bool Q = false;
	bool crossroad = false;
	bool l_byebye = false;
	bool r_byebye = false;
};
}


