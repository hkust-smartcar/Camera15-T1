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
	~ImageProcess()
		{}

	void start(Byte* image);
	float Analyze(void);

	float MidpointSumCal(uint16_t start, uint16_t end);
	void printResult();
	int getState(){return STATE;}

private:

	Car* car;

	//data
	bool bitmap[58][78];
	uint8_t margin[58][2];
	uint8_t midpoint[58];
	uint8_t bias_crossroad_margin[58][4];
	uint8_t data[58][4]; //[0]: number of white pixels; [1]: white row; [2]: black row; [3]: more white at left(0)/right(1)

	//evidence
	uint8_t white_count;
	uint8_t black_end;
	uint8_t checkRA;
	uint8_t white_start;
	uint8_t white_end;

	//for amplifying error
	uint8_t FACTOR;

	//indicate situation
	bool crossroad;
	bool l_byebye;
	bool r_byebye;
	bool right_angle;
	bool black_line;
	bool black_guide_line;
	bool bias_crossroad;

	//for indicate right angle direction
	bool RA_dir;

	//black guide line
	blacklineProcess blp;

	//vertical scanning
	VerticalImageProcess vip;

	//State
	uint8_t STATE;

	//median filter
	Imp medianFilter;


};
}


