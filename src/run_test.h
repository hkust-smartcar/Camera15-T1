/*
 * run_test_app.h
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#pragma once

#include "app.h"
#include <libsc/k60/lcd_typewriter.h>

namespace camera
{

class RunTestApp : public App
{
public:
	explicit RunTestApp(SystemRes *res)
				: App(res)
				  //writer(Lcd_config())
		{}

	void Run() override;
private:
	int16_t Analyze(Byte* image);
	//libsc::k60::LcdTypewriter writer;
	//libsc::k60::LcdTypewriter Lcd_config();

};
}

