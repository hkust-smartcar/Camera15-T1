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

#define SETPOINT 0
#define SERVO_KP 1
#define SERVO_KI 2
#define SERVO_KD 3
#define START_APP 4

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
	writer->SetTextColor(Lcd::kWhite);
	writer->WriteString(String::Format("UST48 %s %d.%03dV", emotion,
			volt / 1000, volt % 1000).c_str());
}

}

void Launcher::Run()
{
	Car *car = GetSystemRes()->car;

	car->GetServo().SetDegree(SERVO_MID_DEGREE-3000);
	System::DelayMs(400);
	car->GetServo().SetDegree(SERVO_MID_DEGREE+3000);
	System::DelayMs(400);
	car->GetServo().SetDegree(SERVO_MID_DEGREE);


	while (true)
	{

		Uint run_app_id = 0;
		// Pop stack to save resources
		{

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
		setParam(run_app_id);
//		StartApp(run_app_id);
	}
}

void Launcher::setParam(const int id){

	Car *car = GetSystemRes()->car;
	bool editing=false;

	//Input setpoint, servo - kp, ki, kd
	// Pop stack to save resources
	{
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
		menu.AddItem(SETPOINT, "SETPOINT");
		menu.AddItem(SERVO_KP, "SERVO KP");
		menu.AddItem(SERVO_KI, "SERVO KI");
		menu.AddItem(SERVO_KD, "SERVO KD");
		menu.AddItem(START_APP,"START");
		menu.Select(0);

		std::function<void(const Timer::TimerInt, const Timer::TimerInt)> printData =
				[&](const Timer::TimerInt request, const Timer::TimerInt)
		{
			car->GetLcd().SetRegion(Lcd::Rect(0, 128, St7735r::GetW(), LcdTypewriter::GetFontH()));
			writer.SetBgColor(Lcd::kWhite);
			writer.SetTextColor(Lcd::kPurple);
			if(editing){
				writer.WriteString(String::Format("%f",data[menu.GetSelectedId()]).c_str());
			}
			else{
				writer.WriteString(" ");
			}
			looper.RunAfter(request, printData);
		};
		looper.RunAfter(30, printData);

		int position_offset = 0;
		Joystick::Config js_config;
		//down
		js_config.listeners[static_cast<int>(Joystick::State::kDown)] =
				[&](const uint8_t)
		{
			if(editing){
				switch(menu.GetSelectedId())
				{

				case SETPOINT:
					data[0] -= 100;
					break;

				case SERVO_KP:
					data[1] -= 0.005;
					break;

				case SERVO_KI:
					data[2] -= 0.005;
					break;

				case SERVO_KD:
					data[3] -= 0.0005;
					break;

				}
			}
			else{
				position_offset = 1;
			}
		};
		js_config.listener_triggers[static_cast<int>(Joystick::State::kDown)] =
				Joystick::Config::Trigger::kDown;
		//up
		js_config.listeners[static_cast<int>(Joystick::State::kUp)] =
				[&](const uint8_t)
		{
			if(editing){
				switch(menu.GetSelectedId())
				{

				case SETPOINT:
					data[0] += 100;
					break;

				case SERVO_KP:
					data[1] += 0.005;
					break;

				case SERVO_KI:
					data[2] += 0.005;
					break;

				case SERVO_KD:
					data[3] += 0.0005;
					break;

				}
			}
			else{
				position_offset = -1;
			}
		};
		js_config.listener_triggers[static_cast<int>(Joystick::State::kUp)] =
				Joystick::Config::Trigger::kUp;

		car->SetJoystickIsr(&js_config);

		Button::Config ok_btn_config;
		ok_btn_config.listener = [&](const uint8_t)
		{

			if(editing){

				switch(menu.GetSelectedId())
				{

				case SETPOINT:
					menu.edit_label(SETPOINT, "SETPOINT");
					break;

				case SERVO_KP:
					menu.edit_label(SERVO_KP, "SERVO KP");
					break;

				case SERVO_KI:
					menu.edit_label(SERVO_KI, "SERVO KI");
					break;

				case SERVO_KD:
					menu.edit_label(SERVO_KD, "SERVO KD");
					break;

				}
				editing = false;

			}
			else{

				switch(menu.GetSelectedId())
				{

				case START_APP:
					looper.Break();
					break;

				default:
					editing = true;
					break;
				}
			}

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
	StartApp(id);

}

void Launcher::StartApp(const int id)
{

	Car *car = GetSystemRes()->car;

	LcdTypewriter::Config writer_conf;
	writer_conf.lcd = &car->GetLcd();
	LcdTypewriter writer(writer_conf);
	writer.SetTextColor(Lcd::kBlack);
	writer.SetBgColor(Lcd::kWhite);

	car->GetAdc().StartConvert();
	car->GetGpo().Set(1);

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

		RunTestApp app(GetSystemRes(),data[0],data[1],data[2],data[3]);

		float adc_result = car->GetAdc().GetResultF();

		while (adc_result<0.5f)
		{
			if(car->GetBuzzer().GetBeep()){
				car->GetBuzzer().SetBeep(false);
			}
			else{
				car->GetBuzzer().SetBeep(true);
			}
			adc_result = car->GetAdc().GetResultF();
			car->GetLcd().SetRegion(Lcd::Rect(0,0,St7735r::GetW(),LcdTypewriter::GetFontH()));
			writer.WriteString(String::Format("Ready...%f",adc_result).c_str());
			System::DelayMs(20);
		}
		app.Run();
	}
	break;
	}
}

}
