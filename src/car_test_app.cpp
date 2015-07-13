/*
 * car_test_app.cpp
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cstdio>
#include <cstring>
#include <memory>

#include <libsc/lcd_typewriter.h>
#include <libsc/k60/led.h>
#include <libsc/k60/ov7725.h>
#include <libsc/st7735r.h>
#include <libsc/system.h>
#include <libsc/timer.h>
#include <libutil/looper.h>
#include <libutil/misc.h>
#include <libutil/string.h>
#include <libsc/k60/jy_mcu_bt_106.h>

#include "car.h"
#include "system_res.h"
#include "car_test_app.h"

using namespace libsc;
using namespace libsc::k60;
using namespace libutil;
using namespace std;

namespace camera
{

void CarTestApp::Run()
{
	printf("CarTestApp\n");

	Car *car = GetSystemRes()->car;
	car->GetLcd().Clear(libutil::GetRgb565(0x33, 0xB5, 0xE5));

	const Uint image_size = car->GetCameraW() * car->GetCameraH() / 8;
	unique_ptr<Byte[]> image2(new Byte[image_size]);

	Looper looper;
	int ec=0;

	looper.Repeat(199, std::bind(&libsc::Led::Switch, &car->GetLed(0)), Looper::RepeatMode::kLoose);

	LcdTypewriter::Config writer_conf;
	writer_conf.lcd = &car->GetLcd();
	writer_conf.bg_color = libutil::GetRgb565(0x33, 0xB5, 0xE5);
	LcdTypewriter writer(writer_conf);

	car->GetLcd().SetRegion({0, 128, St7735r::GetW(), LcdTypewriter::GetFontH()});
	writer.WriteString("Encoder:");
//
//	car->SetMotorPower(0,1000);
//	car->SetMotorPower(1,1000);

	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> encoder =
				[&](const Timer::TimerInt, const Timer::TimerInt)
			{
				car->UpdateAllEncoders();
				ec+=car->GetEncoderCount(1);
#ifdef CAR_WITH_BT
				char buffer[100];
				sprintf(buffer,"%ld    %ld\n",car->GetEncoderCount(0),car->GetEncoderCount(1));
				car->GetUart().SendStr(buffer);
#else
				car->GetLcd().SetRegion({0, 144, St7735r::GetW(),
						LcdTypewriter::GetFontH()});
				writer.WriteString(String::Format("%ld, %ld\n",
						car->GetEncoderCount(0), ec).c_str());
#endif
			};
	looper.Repeat(11, encoder, Looper::RepeatMode::kPrecise);

	car->GetCamera().Start();
	looper.ResetTiming();
	while (!car->GetButton(1).IsDown())
	{
		if (car->GetCamera().IsAvailable())
		{
			memcpy(image2.get(), car->GetCamera().LockBuffer(), image_size);
			car->GetCamera().UnlockBuffer();

			car->GetLcd().SetRegion({0, 0, car->GetCameraW(), car->GetCameraH()});
			car->GetLcd().FillBits(0, 0xFFFF, image2.get(),
					car->GetCameraW() * car->GetCameraH());

			mf.medianFilterPrint(image2.get(), &car->GetLcd());

		}

		looper.Once();
	}

	car->GetCamera().Stop();
}

}
