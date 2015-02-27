/*
 * car.cpp
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cstdint>

#include <libsc/k60/ab_encoder.h>
#include <libsc/k60/dir_motor.h>
#include <libsc/k60/futaba_s3010.h>
#include <libsc/k60/jy_mcu_bt_106.h>
#include <libsc/k60/led.h>
#include <libsc/k60/ov7725.h>
#include <libsc/k60/st7735r.h>

#include "car.h"

using namespace libsc::k60;

namespace camera
{

namespace
{

Button::Config GetButtonConfig(const uint8_t id)
{
	Button::Config product;
	product.id = id;
	if (id == 0)
	{
		product.is_active_low = false;
		product.is_use_pull_resistor = true;
	}
	else
	{
		product.is_active_low = true;
	}
	return product;
}

Ov7725::Config GetOv7725Config()
{
	Ov7725::Config product;
	product.w = 80;
	product.h = 60;
	product.fps = Ov7725::Config::Fps::kLow;
	return product;
}

AbEncoder::Config GetEncoderConfig(const uint8_t id)
{
	AbEncoder::Config product;
	product.id = id;
	return product;
}

Joystick::Config GetJoystickConfig()
{
	Joystick::Config product;
	product.id = 0;
	product.is_active_low = true;
	return product;
}

St7735r::Config GetLcdConfig()
{
	St7735r::Config product;
	product.is_revert = true;
	return product;
}

Led::Config GetLedConfig(const uint8_t id)
{
	Led::Config product;
	product.id = id;
	return product;
}

DirMotor::Config GetMotorConfig(const uint8_t id)
{
	DirMotor::Config product;
	product.id = id;
	return product;
}

FutabaS3010::Config GetServoConfig()
{
	FutabaS3010::Config product;
	product.id = 0;
	return product;
}

JyMcuBt106::Config GetUartConfig()
{
	JyMcuBt106::Config product;
	product.id = 0;
	product.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	product.tx_dma_channel = 0;
	return product;
}

}

Car::Car()
		: m_buttons{Button(GetButtonConfig(0)), Button(GetButtonConfig(1))},
		  m_camera(GetOv7725Config()),
		  m_encoders{AbEncoder(GetEncoderConfig(0)),
				AbEncoder(GetEncoderConfig(1))},
		  m_joystick(GetJoystickConfig()),
		  m_lcd(GetLcdConfig()),
		  m_leds{Led(GetLedConfig(0)), Led(GetLedConfig(1)), Led(GetLedConfig(2)),
				Led(GetLedConfig(3))},
		  m_motors{DirMotor(GetMotorConfig(0)), DirMotor(GetMotorConfig(1))},
		  m_servo(GetServoConfig()),
		  m_uart(GetUartConfig())
{}

Car::~Car()
{}

}
