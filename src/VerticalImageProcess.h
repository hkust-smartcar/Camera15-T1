/*
 * VerticalImageProcess.h
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */
#include "libutil/misc.h"
#include <libsc/system.h>
#include <algorithm>
#include "libsc/k60/jy_mcu_bt_106.h"
#include "car.h"

#include "definition.h"

#pragma once

using namespace libsc::k60;
using namespace libsc;
using namespace std;
using namespace libutil;


namespace camera
{

class VerticalImageProcess{

public:
	VerticalImageProcess();
	~VerticalImageProcess()
	{}
	void Analyze(bool binary[HEIGHT][WIDTH]);

	bool detected_black_line();
	bool detected_right_angle();

	uint8_t get_check_row(){return libutil::Clamp(CS,highest_cont_margin-5,CE);}
	uint8_t get_highest_cont_margin(){return highest_cont_margin;}
	uint8_t get_lowest_cont_margin(){return lowest_cont_margin;}
	uint8_t get_top_len(){return top_len;}
	uint8_t get_bottom_len(){return bottom_len;}

private:
	uint8_t margin[WIDTH][2];

	uint8_t cont_top_margin[WIDTH][3];
	uint8_t cont_bottom_margin[WIDTH][3];

	uint8_t cont_top_count = 0;
	uint8_t cont_bottom_count = 0;

	uint8_t threshold = 15;

	uint8_t top_len = 0;
	uint8_t bottom_len = 0;

	uint8_t highest_cont_margin = CE-1;
	uint8_t lowest_cont_margin = CS;

	uint8_t top_cont_last = 0;
	uint8_t bottom_cont_last = 0;

};

}
