/*
 * camera_test_app.cpp
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cstdio>

#include <libsc/k60/led.h>
#include <libsc/k60/ov7725.h>
#include <libsc/k60/st7735r.h>
#include <libsc/k60/system.h>
#include <libutil/misc.h>

#include "camera_test_app.h"

using namespace libsc::k60;

namespace camera
{

void CameraTestApp::Run()
{
	printf("OV7725 Test\n");

	Led leds[] = {Led({0}), Led({1}), Led({2}), Led({3})};
	St7735r::Config lcd_config;
	lcd_config.is_revert = true;
	St7735r lcd(lcd_config);
	lcd.Clear(libutil::GetRgb565(0x33, 0xB5, 0xE5));

	Ov7725::Config camera_config;
	camera_config.w = 80;
	camera_config.h = 60;
	camera_config.fps = Ov7725::Config::Fps::kLow;
	Ov7725 camera(camera_config);

	camera.Start();
	while (true)
	{
		leds[0].Switch();
		if (camera.IsAvailable())
		{
			const Byte *image = camera.LockBuffer();
			lcd.SetRegion({0, 0, 80, 60});
			lcd.FillBits(0, 0xFFFF, image, 80 * 60);
			camera.UnlockBuffer();
		}
		System::DelayMs(150);
	}
}

}
