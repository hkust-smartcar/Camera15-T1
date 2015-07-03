/*
 * launcher.cpp
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <functional>

#include <libbase/k60/adc.h>
#include <libsc/button.h>
#include <libsc/joystick.h>
#include <libsc/lcd_typewriter.h>
#include <libsc/k60/led.h>
#include <libsc/st7735r.h>
#include <libutil/looper.h>
#include <libutil/misc.h>
#include <libutil/string.h>

#include "car.h"
#include "car_test_app.h"
#include "run_test.h"

#include "launcher.h"
#include "lcd_menu.h"
#include "system_res.h"

using namespace libsc;
using namespace libsc::k60;
using namespace libutil;

#define NORMAL_ID 0
#define CAR_TEST_ID 1
#define RUN_TEST_ID 2

namespace camera
{

namespace
{

void PrintTitleLine(Car *car, LcdTypewriter *writer)
{
	car->GetLcd().SetRegion({0, 0, St7735r::GetW(), LcdTypewriter::GetFontH()});
	const uint16_t volt = Clamp<Uint>(7200, car->GetBatteryMeter().GetVoltage()
			* 1000, 8200);
	const uint16_t supplement = (volt - 7200) / 10;
	const uint16_t color = GetRgb565(255 - supplement, 155, 155 + supplement);

	const char *emotion = nullptr;
	if (volt < 7300)
	{
		emotion = "T^T";
	}
	else if (volt < 7400)
	{
		emotion = ">.<";
	}
	else if (volt < 7600)
	{
		emotion = "-_-";
	}
	else
	{
		emotion = "^3^";
	}
	writer->SetBgColor(color);
	writer->WriteString(String::Format("UST48 %s %d.%03dV", emotion,
			volt / 1000, volt % 1000).c_str());
}

}

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

			LcdTypewriter::Config writer_conf;
			writer_conf.lcd = &car->GetLcd();
			LcdTypewriter writer(writer_conf);
			std::function<void(const Timer::TimerInt, const Timer::TimerInt)> battery =
					[&](const Timer::TimerInt request, const Timer::TimerInt)
					{
				PrintTitleLine(car, &writer);
				looper.RunAfter(request, battery);
					};
			looper.RunAfter(250, battery);

			car->GetLcd().Clear(0);

			car->GetLcd().SetRegion({0, LcdTypewriter::GetFontH(), St7735r::GetW(),
				St7735r::GetH() - LcdTypewriter::GetFontH()});
			LcdMenu menu(&car->GetLcd());
			//menu.AddItem(NORMAL_ID, "Normal");
			menu.AddItem(CAR_TEST_ID, "Car Test");
			menu.AddItem(RUN_TEST_ID, "Run Test");
			menu.Select(0);

			int position_offset = 0;
			Joystick::Config js_config;
			js_config.listeners[static_cast<int>(Joystick::State::kDown)] =
					[&](const uint8_t)
					{
				position_offset = 1;
					};
			js_config.listener_triggers[static_cast<int>(Joystick::State::kDown)] =
					Joystick::Config::Trigger::kDown;
			js_config.listeners[static_cast<int>(Joystick::State::kUp)] =
					[&](const uint8_t)
					{
				position_offset = -1;
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

			looper.ResetTiming();
			while (!looper.IsBreak())
			{
				if (position_offset)
				{
					menu.Select(menu.GetSelectedPosition() + position_offset);
					position_offset = 0;
				}
				looper.Once();
			}
			car->SetJoystickIsr(nullptr);
			car->SetButtonIsr(0, nullptr);
		}
		StartApp(run_app_id);
	}
}


//Gpi::Config getIR(Gpi::OnGpiEventListener isr)
//{
//	Gpi::Config config;
//	config.pin = Pin::Name::kPtc10;
//	config.interrupt = Pin::Config::Interrupt::kHigh;
//	config.isr = isr;
//	return config;
//
//}

void Launcher::StartApp(const int id)
{

	Car *car = GetSystemRes()->car;

	LcdTypewriter::Config writer_conf;
	writer_conf.lcd = &car->GetLcd();
	LcdTypewriter writer(writer_conf);
	writer.SetTextColor(Lcd::kBlack);
	writer.SetBgColor(Lcd::kWhite);

	Adc::Config adc_c;
	adc_c.adc = Adc::Name::kAdc0Ad8;
	adc_c.resolution = Adc::Config::Resolution::k8Bit;
	adc_c.speed = Adc::Config::SpeedMode::kExFast;

	Adc adc(adc_c);
	adc.StartConvert();

	car->GetLcd().Clear(St7735r::kWhite);

	switch (id)
	{
	case NORMAL_ID:
		break;

	case CAR_TEST_ID:
	{
		CarTestApp app(GetSystemRes());
		app.Run();
	}
	break;
	case RUN_TEST_ID:
	{
		RunTestApp app(GetSystemRes());

//		Gpi IR(getIR([&] (Gpi*)
//				{
//					app.Run();
//				}));
//
		while (adc.GetResultF()<1.0f)
		{
			car->GetLcd().SetRegion(Lcd::Rect(0,0,St7735r::GetW(),LcdTypewriter::GetFontH()));
			writer.WriteString(String::Format("Ready...\n Reading: %f",adc.GetResultF()).c_str());
//			System::DelayMs(20);

		}
		app.Run();
	}
	break;
	}
}

}
