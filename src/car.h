/*
 * car.h
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#pragma once

#include <cstdint>

#include <libsc/ab_encoder.h>
#include <libsc/battery_meter.h>
#include <libsc/button.h>
#include <libsc/dir_motor.h>
#include <libsc/alternate_motor.h>
#include <libsc/futaba_s3010.h>
#include <libsc/joystick.h>
#include <libsc/k60/jy_mcu_bt_106.h>
#include <libsc/led.h>
#include <libsc/k60/ov7725.h>
#include <libsc/st7735r.h>

#include "MyVarManager.h"

namespace camera
{

class Car
{
public:
	Car();
	~Car();

	void UpdateAllEncoders()
	{
		m_encoders[0].Update();
		m_encoders[1].Update();
	}

	/**
	 * Return the encoder count, a positive count means the corresponding wheel
	 * going forward
	 *
	 * @param id
	 * @return
	 */
	int32_t GetEncoderCount(const uint8_t id);

	/**
	 * Set the power of the motor, a negative power will drive the corresponding
	 * wheel backwards
	 *
	 * @param id
	 * @param power Power scale in [-1000, 1000]
	 */
	void SetMotorPower(const uint8_t id, const int16_t power);

	/**
	 * Set the turning percentage, negative input means turning right
	 *
	 * @param percentage Specifying how aggressively should the car turn,
	 * in [-1000, 1000], where passing 0 basically means going straight
	 */
	void SetTurning(const int16_t percentage);

	libsc::BatteryMeter& GetBatteryMeter()
	{
		return m_battery;
	}

	libsc::Button& GetButton(const uint8_t id)
	{
		return m_buttons[id];
	}

	libsc::k60::Ov7725& GetCamera()
	{
		return m_camera;
	}

	libsc::AbEncoder& GetEncoder(const uint8_t id)
	{
		return m_encoders[id];
	}

	libsc::Joystick& GetJoystick()
	{
		return m_joystick;
	}

	libsc::St7735r& GetLcd()
	{
		return m_lcd;
	}

	libsc::Led& GetLed(const uint8_t id)
	{
		return m_leds[id];
	}

	libsc::DirMotor& GetMotor(const uint8_t id)
	{
		return m_motors[id];
	}

//	libsc::AlternateMotor& GetMotor(const uint8_t id)
//	{
//		return m_motors[id];
//	}


	libsc::FutabaS3010& GetServo()
	{
		return m_servo;
	}

//	libsc::k60::JyMcuBt106& GetUart()
//	{
//		return m_uart;
//	}

	/**
	 * Set interrupt for the button, effectively it will reinit the button. Only
	 * libsc::Button::Config::listener and
	 * libsc::Button::Config::listener_trigger are considered. If @a config
	 * is nullptr, it'll disable the interrupt
	 *
	 * @param id
	 * @param config
	 */
	void SetButtonIsr(const uint8_t id, const libsc::Button::Config *config);

	/**
	 * Set interrupt for the joystick, effectively it will reinit the joystick.
	 * Only libsc::Joystick::Config::listeners and
	 * libsc::Joystick::Config::listener_triggers are considered. If
	 * @a config is nullptr, it'll disable the interrupt
	 *
	 * @param config
	 */
	void SetJoystickIsr(const libsc::Joystick::Config *config);

	void SetUartIsr(const libsc::k60::UartDevice::OnReceiveListener &isr);

	static constexpr int GetCameraW()
	{
//		return 80;
		return 78;
	}

	static constexpr int GetCameraH()
	{
//		return 60;
		return 58;
	}

private:
	libsc::BatteryMeter m_battery;
	libsc::Button m_buttons[2];
	libsc::k60::Ov7725 m_camera;
	libsc::AbEncoder m_encoders[2];
	libsc::Joystick m_joystick;
	libsc::St7735r m_lcd;
	libsc::Led m_leds[4];
	libsc::DirMotor m_motors[2];
//	libsc::AlternateMotor m_motors[2];
	libsc::FutabaS3010 m_servo;
//	libsc::k60::JyMcuBt106 m_uart;
};

}
