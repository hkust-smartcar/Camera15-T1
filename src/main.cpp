/*
 * main.cpp
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <libbase/k60/mcg.h>

#include <libsc/lib_guard.h>
#include <libsc/system.h>
#include <libutil/looper.h>
#include <libutil/misc.h>

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

	libsc::System::Init();

	Car car;
	SystemRes res;
	res.car = &car;

	Launcher launcher(&res);
	launcher.Run();

//	JyMcuBt106::Config bt_conf;
//		bt_conf.id = 0;
//		bt_conf.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
//		JyMcuBt106 bt(bt_conf);
//		libutil::InitDefaultFwriteHandler(&bt);
//
//		libutil::Looper looper;
//
//		std::function<void(const Timer::TimerInt, const Timer::TimerInt)> blink =
//				[&](const Timer::TimerInt request, const Timer::TimerInt)
//				{
//					res.car->GetLed(0).Switch();
//					printf("%d, I am led looper\n",(int)System::Time());
//
//				};
//		looper.Repeat(199, blink, libutil::Looper::RepeatMode::kLoose);
//
//		std::function<void(const Timer::TimerInt, const Timer::TimerInt)> encoder =
//				[&](const Timer::TimerInt request, const Timer::TimerInt)
//				{
//					res.car->UpdateAllEncoders();
//					printf("%d, I am encoder looper\n",(int)System::Time());
//				};
//		looper.Repeat(19, encoder, libutil::Looper::RepeatMode::kPrecise);
//
//	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> data =
//				[&](const Timer::TimerInt request, const Timer::TimerInt)
//				{
//					printf("%d, I am data looper\n",(int)System::Time());
//				};
//		looper.Repeat(31, data, libutil::Looper::RepeatMode::kLoose);
//
//	looper.ResetTiming();
//	while(1){
//		looper.Once();
//	}

	return 0;
}
