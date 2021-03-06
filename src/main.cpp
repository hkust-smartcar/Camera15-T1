/*
 * main.cpp
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <libbase/k60/mcg.h>

#include <libsc/lib_guard.h>
#include <libsc/k60/system.h>

#include "car.h"
#include "launcher.h"
#include "system_res.h"

using namespace camera;

namespace libbase
{
namespace k60
{

Mcg::Config Mcg::GetMcgConfig()
{
	Mcg::Config config;
	config.external_oscillator_khz = 50000;
	//config.core_clock_khz = 150000;
	return config;
}

}
}

int main()
{
	LIBSC_GUARD();

	libsc::k60::System::Init();

	Car car;
	SystemRes res;
	res.car = &car;

	Launcher launcher(&res);
	launcher.Run();
	return 0;
}
