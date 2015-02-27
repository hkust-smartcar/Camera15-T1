/*
 * launcher.cpp
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <functional>

#include <libsc/k60/button.h>
#include <libsc/k60/joystick.h>
#include <libsc/k60/led.h>
#include <libsc/k60/st7735r.h>
#include <libutil/looper.h>

#include "camera_test_app.h"
#include "car.h"
#include "launcher.h"
#include "lcd_menu.h"
#include "system_res.h"

using namespace libsc::k60;
using namespace libutil;

#define NORMAL_ID 0
#define CAMERA_TEST_ID 1

namespace camera
{

void Launcher::Run()
{
	while (true)
	{
		Uint run_app_id = 0;
		// Pop stack to save resources
		{
			Car *car = GetSystemRes()->car;
			Looper looper;

			std::function<void(const Timer::TimerInt, const Timer::TimerInt)> blink =
					[&](const Timer::TimerInt request, const Timer::TimerInt)
					{
						car->GetLed(0).Switch();
						looper.RunAfter(request, blink);
					};
			looper.RunAfter(200, blink);

			car->GetLcd().Clear(0);

			LcdMenu menu(&car->GetLcd());
			//menu.AddItem(NORMAL_ID, "Normal");
			menu.AddItem(CAMERA_TEST_ID, "Camera Test");
			menu.Select(0);

			Joystick::Config js_config;
			js_config.listeners[static_cast<int>(Joystick::State::kDown)] =
					[&](const uint8_t)
					{
						menu.Select(menu.GetSelectedPosition() + 1);
					};
			js_config.listener_triggers[static_cast<int>(Joystick::State::kDown)] =
					Joystick::Config::Trigger::kDown;
			js_config.listeners[static_cast<int>(Joystick::State::kUp)] =
					[&](const uint8_t)
					{
						menu.Select(static_cast<int>(menu.GetSelectedPosition()) - 1);
					};
			js_config.listener_triggers[static_cast<int>(Joystick::State::kUp)] =
					Joystick::Config::Trigger::kDown;
			car->SetJoystickIsr(&js_config);

			Button::Config ok_btn_config;
			ok_btn_config.listener = [&](const uint8_t)
					{
						run_app_id = menu.GetSelectedId();
						looper.Break();
					};
			ok_btn_config.listener_trigger = Button::Config::Trigger::kDown;
			car->SetButtonIsr(0, &ok_btn_config);

			looper.Loop();
			car->SetJoystickIsr(nullptr);
			car->SetButtonIsr(0, nullptr);
		}
		StartApp(run_app_id);
	}
}

void Launcher::StartApp(const int id)
{
	switch (id)
	{
	case NORMAL_ID:
		break;

	case CAMERA_TEST_ID:
		{
			CameraTestApp app(GetSystemRes());
			app.Run();
		}
		break;
	}
}

}
