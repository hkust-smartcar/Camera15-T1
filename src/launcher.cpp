/*
 * launcher.cpp
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <functional>

#include <libsc/k60/button.h>
#include <libsc/k60/led.h>
#include <libsc/k60/st7735r.h>
#include <libutil/looper.h>

#include "camera_test_app.h"
#include "launcher.h"
#include "lcd_menu.h"

using namespace libsc::k60;
using namespace libutil;

#define NORMAL_ID 0
#define CAMERA_TEST_ID 1

namespace camera
{

void Launcher::Run()
{
	Uint run_app_id = 0;
	// Peripherals can't be reinited, so we have to kill them after use here
	{
		Looper looper;

		Led leds[] = {Led({0}), Led({1}), Led({2}), Led({3})};
		std::function<void(const Timer::TimerInt, const Timer::TimerInt)> blink =
				[&](const Timer::TimerInt request, const Timer::TimerInt)
				{
					leds[0].Switch();
					looper.RunAfter(request, blink);
				};
		looper.RunAfter(200, blink);

		St7735r::Config lcd_config;
		lcd_config.is_revert = true;
		St7735r lcd(lcd_config);
		lcd.Clear(0);

		LcdMenu menu(&lcd);
		//menu.AddItem(NORMAL_ID, "Normal");
		menu.AddItem(CAMERA_TEST_ID, "Camera Test");
		menu.Select(0);

		// TODO Joystick to select between items

		Button::Config ok_btn_config;
		ok_btn_config.id = 0;
		ok_btn_config.is_active_low = true;
		ok_btn_config.is_use_pull_resistor = true;
		ok_btn_config.listener = [&](const uint8_t)
				{
					run_app_id = menu.GetSelectedId();
					looper.Break();
				};
		ok_btn_config.listener_trigger = Button::Config::Trigger::kDown;
		Button ok_btn(ok_btn_config);

		looper.Loop();
	}
	StartApp(run_app_id);
}

void Launcher::StartApp(const int id)
{
	switch (id)
	{
	case NORMAL_ID:
		break;

	case CAMERA_TEST_ID:
		{
			CameraTestApp app;
			app.Run();
		}
		break;
	}
}

}
