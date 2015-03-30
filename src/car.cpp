/*
 * car.cpp
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cstdint>

#include <libsc/ab_encoder.h>
#include <libsc/battery_meter.h>
#include <libsc/button.h>
#include <libsc/dir_motor.h>
#include <libsc/futaba_s3010.h>
#include <libsc/k60/jy_mcu_bt_106.h>
#include <libsc/k60/led.h>
#include <libsc/k60/ov7725.h>
#include <libsc/st7735r.h>
#include <libutil/misc.h>

#include "car.h"

using namespace libsc::k60;
using namespace libsc;
using namespace libutil;

#define SERVO_MID_DEGREE 950
#define SERVO_AMPLITUDE 370

namespace camera
{

namespace
{

BatteryMeter::Config GetBatteryMeterConfig()
{
	BatteryMeter::Config product;
	product.voltage_ratio = 0.39;
	return product;
}

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
	product.w = Car::GetCameraW();
	product.h = Car::GetCameraH();
	product.fps = Ov7725::Config::Fps::kLow;
	product.contrast = 0x40;
	return product;
}

AbEncoder::Config GetEncoderConfig(const uint8_t id)
{
	AbEncoder::Config product;
	product.id = id;
	return product;
}

libsc::Joystick::Config GetJoystickConfig()
{
	libsc::Joystick::Config product;
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

libsc::Led::Config GetLedConfig(const uint8_t id)
{
	libsc::Led::Config product;
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
	//product.tx_dma_channel = 0;
	return product;
}

}

Car::Car()
		: m_battery(GetBatteryMeterConfig()),
		  m_buttons{Button(GetButtonConfig(0)), Button(GetButtonConfig(1))},
		  m_camera(GetOv7725Config()),
		  m_encoders{AbEncoder(GetEncoderConfig(0)),
				AbEncoder(GetEncoderConfig(1))},
		  m_joystick(GetJoystickConfig()),
		  m_lcd(GetLcdConfig()),
		  m_leds{libsc::Led(GetLedConfig(0)), libsc::Led(GetLedConfig(1)), libsc::Led(GetLedConfig(2)),
					libsc::Led(GetLedConfig(3))},
		  m_motors{DirMotor(GetMotorConfig(0)), DirMotor(GetMotorConfig(1))},
		  m_servo(GetServoConfig()),
		  m_uart(GetUartConfig())
{
	m_servo.SetDegree(SERVO_MID_DEGREE);
}

Car::~Car()
{}

int32_t Car::GetEncoderCount(const uint8_t id)
{
	if (id == 0)
	{
		return m_encoders[id].GetCount();
	}
	else
	{
		return -m_encoders[id].GetCount();
	}
}

void Car::SetMotorPower(const uint8_t id, const int16_t power)
{
	const Uint power_ = Clamp<Uint>(0, abs(power), 1000);
	m_motors[id].SetClockwise((power < 0) ^ (id == 0));
	m_motors[id].SetPower(power_);
}

void Car::SetTurning(const int16_t percentage)
{
	const int percentage_ = libutil::Clamp<int>(-1000, percentage, 1000);
	const int degree = SERVO_MID_DEGREE + (percentage_ * SERVO_AMPLITUDE / 1000);
	m_servo.SetDegree(degree);
	//m_servo.SetDegree(percentage);
}

void Car::SetButtonIsr(const uint8_t id, const Button::Config *config)
{
	auto btn_config = GetButtonConfig(id);
	if (config)
	{
		btn_config.listener = config->listener;
		btn_config.listener_trigger = config->listener_trigger;
	}
	m_buttons[id] = Button(nullptr);
	m_buttons[id] = Button(btn_config);
}

void Car::SetJoystickIsr(const libsc::Joystick::Config *config)
{
	auto js_config = GetJoystickConfig();
	if (config)
	{
		for (int i = 0; i < 5; ++i)
		{
			js_config.listeners[i] = config->listeners[i];
			js_config.listener_triggers[i] = config->listener_triggers[i];
		}
	}
	m_joystick = libsc::Joystick(nullptr);
	m_joystick = libsc::Joystick(js_config);
}

}
