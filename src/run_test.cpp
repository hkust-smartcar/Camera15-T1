/*
 * run_test_app.cpp
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cstdio>
#include <cstring>
#include <memory>
#include <cmath>

#include <libsc/k60/led.h>
#include <libsc/k60/ov7725.h>
#include <libsc/st7735r.h>
#include <libsc/system.h>
#include <libsc/timer.h>
#include <libutil/looper.h>
#include <libutil/misc.h>
#include "libutil/misc.h"
#include "libsc/joystick.h"

#include "car.h"
#include "system_res.h"
#include "run_test.h"
#include "ImageProcess.h"


#include "libbase/k60/pit.h"
#include "libbase/k60/clock_utils.h"

using namespace libsc;
using namespace libsc::k60;
using namespace libutil;
using namespace std;

namespace camera
{

RunTestApp *m_instance;

RunTestApp::RunTestApp(SystemRes *res)
: App(res),

  s_kp(0.38f), //0.4, 0.03
  s_ki(0.0f),
  s_kd(0.045f),

  s_setpoint(0.0f),

  //19ms
  l_kp(0.0125f),
  l_ki(0.0f),
  l_kd(0.0004f),

  r_kp(0.0099f),
  r_ki(0.0f),
  r_kd(0.0004f),

  //19 ms
  l_m_setpoint(1500.0f), //2900
  r_m_setpoint(1500.0f),

  sd_setpoint(1500),

  show_error(0.0),

  RA_time(System::Time()),

  servo_Control(&s_setpoint, &s_kp, &s_ki, &s_kd, -1000, 1000),
  l_speedControl(&l_m_setpoint, &l_kp, &l_ki, &l_kd, 0, 950),
  r_speedControl(&r_m_setpoint, &r_kp, &r_ki, &r_kd, 0, 950),

  m_peter(),

  imageProcess(GetSystemRes()->car),

  m_start(0),
  m_is_stop(false)

{
	//raw data: left & right encoder, servo angle
	ec0 = 0;
	ec1 = 0;
	s_degree = 0;

	l_result = 0;
	r_result = 0;
	s_result = 0;

	sp_storage[0] = l_m_setpoint;
	sp_storage[1] = r_m_setpoint;

	m_instance = this;

//for grapher use
	m_peter.addWatchedVar(&sd_setpoint, "sd_setpoint");
	m_peter.addWatchedVar(&show_error, "error");

// servo
	m_peter.addSharedVar(&s_kp,"skp");
//	m_peter.addSharedVar(&s_ki,"ski");
	m_peter.addSharedVar(&s_kd,"skd");

	m_peter.Init(&PeggyListener);
}

//void RunTestApp::updateSPD(float error){
//
//}

void RunTestApp::DetectEmergencyStop(){

	//Get car
	Car *car = GetSystemRes()->car;

	static bool is_startup = true;
	const Timer::TimerInt time = System::Time();
	if (is_startup && Timer::TimeDiff(time, m_start) > 2000)
	{
		is_startup = false;
	}

	const int count = car->GetEncoderCount(0);
	const int count2 = car->GetEncoderCount(1);
	if (!is_startup && (abs(count) + abs(count2) < 60))
	{
		if (m_emergency_stop_state.is_triggered)
		{
			if (Timer::TimeDiff(time, m_emergency_stop_state.trigger_time) > 150)
			{
				// Emergency stop
				m_is_stop = true;
			}
		}
		else
		{
			m_emergency_stop_state.is_triggered = true;
			m_emergency_stop_state.trigger_time = time;
		}
	}
	else
	{
		m_emergency_stop_state.is_triggered = false;
	}

}

void RunTestApp::Run()
{
	printf("CarTestApp\n");

	//Get car
	Car *car = GetSystemRes()->car;

	//Get LCD
	car->GetLcd().Clear(libutil::GetRgb565(0x33, 0xB5, 0xE5));

	//Get image size (80*60)/byte_size
	const uint16_t image_size = car->GetCameraW() * car->GetCameraH() / 8;
	unique_ptr<Byte[]> image2(new Byte[image_size]);
//	Pit m_pit(GetPitConfig(0, std::bind(&RunTestApp::peggy, this, std::placeholders::_1)));

	Looper looper;

	//Update encoder count, input to and update speed PID controller
		std::function<void(const Timer::TimerInt, const Timer::TimerInt)> encoder =
				[&](const Timer::TimerInt, const Timer::TimerInt)
				{
					//update and get encoder's count
					car->UpdateAllEncoders();
					ec0 =  car->GetEncoderCount(0);
					ec1 =  car->GetEncoderCount(1);

					speed = ec0/57300.0f/0.019f;

					//if t is true, update PID and give power to car, else stop the car
					if(t && !m_is_stop){
						l_result = (int32_t)l_speedControl.updatePID((float)car->GetEncoderCount(0));
						car->SetMotorPower(0,l_result);

						r_result = (int32_t)r_speedControl.updatePID((float)car->GetEncoderCount(1));
						car->SetMotorPower(1,r_result);

						DetectEmergencyStop();
					}
					else {
						car->SetMotorPower(0,0);
						car->SetMotorPower(1,0);
					}
					//
				};
		looper.Repeat(19, encoder, Looper::RepeatMode::kLoose);
	//	looper.Repeat(89, encoder, Looper::RepeatMode::kPrecise);


	//breathing led- indicate if program hang or not
	looper.Repeat(199, std::bind(&libsc::Led::Switch, &car->GetLed(0)), Looper::RepeatMode::kLoose);

	//Send data to grapher
	looper.Repeat(31, std::bind(&MyVarManager::sendWatchData, &m_peter), Looper::RepeatMode::kLoose);


	//Update servo error, input to and update servo PID controller
	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> servo =
			[&](const Timer::TimerInt, const Timer::TimerInt)
	{

		if (car->GetCamera().IsAvailable())
		{
			memcpy(image2.get(), car->GetCamera().LockBuffer(), image_size);
			car->GetCamera().UnlockBuffer();
		}

			//start image processing
			imageProcess.start(image2.get());

			float error = imageProcess.Analyze();
//			imageProcess.printResult();

			show_error = error;

			//set angle with servo PID controller and image process result
			//negative for correcting direction
			s_result = (int16_t)servo_Control.updatePID_ori(-error);
			car->SetTurning(s_result);

			//software differential
			l_m_setpoint =  (float)software_differential.turn_left_encoder(sd_setpoint,car->GetServo().GetDegree(),SERVO_MID_DEGREE,3700);
			r_m_setpoint = (float) software_differential.turn_right_encoder(sd_setpoint,car->GetServo().GetDegree(),SERVO_MID_DEGREE,3700);

	};
	looper.Repeat(11, servo, Looper::RepeatMode::kPrecise);


	//Get image
	car->GetCamera().Start();
	looper.ResetTiming();
	while (!car->GetButton(1).IsDown())
	{
		looper.Once();
	}
	car->GetCamera().Stop();

}

RunTestApp &getInstance(void)
{
	return *m_instance;
}

//Bluetooth control
void RunTestApp::PeggyListener(const std::vector<Byte> &bytes)
{

	switch (bytes[0])
	{
	//move & stop
	case 'f':
		//reset pid
		m_instance->l_speedControl.reset();
		m_instance->r_speedControl.reset();
		m_instance->servo_Control.reset();

		if(m_instance->t)
			m_instance->t = false;
		else{

			m_instance->m_start = System::Time();
			m_instance->m_is_stop = false;
			m_instance->t = true;
		}
		break;

	//faster!
	case 'r':
		m_instance->l_m_setpoint += 100.0f;
		m_instance->r_m_setpoint += 100.0f;
		m_instance->sd_setpoint += 100.0f;
		break;

	//slower!
	case 'R':
		m_instance->l_m_setpoint -= 100.0f;
		m_instance->r_m_setpoint -= 100.0f;
		m_instance->sd_setpoint -= 100.0f;
		break;
	}
}


}
