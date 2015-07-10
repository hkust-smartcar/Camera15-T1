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

#include "definition.h"

#pragma once

using namespace libsc::k60;
using namespace libsc;
using namespace std;

namespace camera
{
class blacklineProcess{

public:
	blacklineProcess()
	{}
	~blacklineProcess(){}

	float Analyze(bool binary[HEIGHT][WIDTH]);
	bool detected();
	bool approaching();
	uint8_t get_nearest_blackGuideLine(){return nearest_blackGuideLine;}
	uint8_t get_margin(int i, int j){return margin[i][j];}
	uint8_t get_midpoint(int i){return midpoint[i];}
	uint8_t get_narrow_count(){return narrow_count;}

private:
	uint8_t midpoint[HEIGHT];
	uint8_t margin[HEIGHT][2];
	uint8_t narrow_count=0;
	uint8_t nearest_blackGuideLine = 0;

};
}
