/*
 * car_test_app.h
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#pragma once

#include "app.h"
#include "Imp.h"

namespace camera
{

class CarTestApp : public App
{
public:
	explicit CarTestApp(SystemRes *res)
			: App(res)
	{}

	void Run() override;

	Imp mf;

};

}
