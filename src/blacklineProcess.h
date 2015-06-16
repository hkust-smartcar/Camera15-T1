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

#pragma once

using namespace libsc::k60;
using namespace libsc;
using namespace std;

#define RS 0	//row start
#define RE 79	//row end

#define HEIGHT 60
#define WIDTH 80


#define LEFT 0
#define RIGHT 1

namespace camera
{
class blacklineProcess{

public:
	blacklineProcess()
	{}
	~blacklineProcess(){}

	int16_t Analyze(bool binary[60][80]);
	bool detected();

	uint16_t midpoint[60];
	uint16_t margin[60][2];
	uint16_t narrow_count=0;

};
}
