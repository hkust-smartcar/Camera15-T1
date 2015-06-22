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

	float Analyze(bool binary[58][78]);
	bool detected();
	bool approaching();

	uint16_t midpoint[60];
	uint16_t margin[60][2];
	uint16_t narrow_count=0;
	uint16_t nearest_blackGuideLine = 0;

};
}
