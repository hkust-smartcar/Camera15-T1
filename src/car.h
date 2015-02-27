/*
 * car.h
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#pragma once

#include <libsc/k60/ab_encoder.h>
#include <libsc/k60/button.h>
#include <libsc/k60/dir_motor.h>
#include <libsc/k60/futaba_s3010.h>
#include <libsc/k60/joystick.h>
#include <libsc/k60/jy_mcu_bt_106.h>
#include <libsc/k60/led.h>
#include <libsc/k60/ov7725.h>
#include <libsc/k60/st7735r.h>

namespace camera
{

class Car
{
public:
	Car();
	~Car();

	static constexpr int GetCameraW()
	{
		return 80;
	}

	static constexpr int GetCameraH()
	{
		return 60;
	}

private:
	libsc::k60::Button m_buttons[2];
	libsc::k60::Ov7725 m_camera;
	libsc::k60::AbEncoder m_encoders[2];
	libsc::k60::Joystick m_joystick;
	libsc::k60::St7735r m_lcd;
	libsc::k60::Led m_leds[4];
	libsc::k60::DirMotor m_motors[2];
	libsc::k60::FutabaS3010 m_servo;
	libsc::k60::JyMcuBt106 m_uart;
};

}
