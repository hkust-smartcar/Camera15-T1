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
#include <libsc/simple_buzzer.h>

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

RunTestApp::RunTestApp(SystemRes *res, uint16_t motor_setpoint, float skp, float ski, float skd)
: App(res),

  s_kp(skp), //0.4, 0.03
  s_ki(ski),
  s_kd(skd),

  s_setpoint(0.0f),

  //19ms
  l_kp(0.0108f),
  l_ki(0.0f),
  l_kd(0.00055f),

  r_kp(0.0104f),
  r_ki(0.0f),
  r_kd(0.00055f),

  //19 ms
  l_m_setpoint(motor_setpoint), //2900
  r_m_setpoint(motor_setpoint),

  sd_setpoint((uint16_t)motor_setpoint),

  show_error(0.0),

  RA_time(System::Time()),

  servo_Control(&s_setpoint, &s_kp, &s_ki, &s_kd, -1000, 1000),
  l_speedControl(&l_m_setpoint, &l_kp, &l_ki, &l_kd, 0, 950),
  r_speedControl(&r_m_setpoint, &r_kp, &r_ki, &r_kd, 0, 950),

  imageProcess(GetSystemRes()->car),

  m_start(0),
  m_is_stop(false),

  m_peter(),

  prev_adc(0),
  gpo(0)


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
//	m_peter.addWatchedVar(&l_m_setpoint, "Left SP");
//	m_peter.addWatchedVar(&r_m_setpoint, "Right SP");

//	m_peter.addWatchedVar(&show_error, "error");
//	m_peter.addWatchedVar(&s_kp,"skp");
//	m_peter.addWatchedVar(&s_ki,"ski");
//	m_peter.addWatchedVar(&s_kd,"skd");
	m_peter.addWatchedVar(&ec0,"left error");
	m_peter.addWatchedVar(&ec1,"right error");
//	m_peter.addWatchedVar(&prev_adc, "adc");
//	m_peter.addWatchedVar(&gpo, "gpo");

// servo
	m_peter.addSharedVar(&s_kp,"skp");
//	m_peter.addSharedVar(&s_ki,"ski");
	m_peter.addSharedVar(&s_kd,"skd");

	m_peter.Init(&PeggyListener);
	prev_adc = GetSystemRes()->car->GetAdc().GetResultF();

}

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
	if (!is_startup && t && (abs(count) + abs(count2) < 60))
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

	Looper looper;
	m_start = System::Time();

	//Update encoder count, input to and update speed PID controller
		std::function<void(const Timer::TimerInt, const Timer::TimerInt)> encoder =
				[&](const Timer::TimerInt, const Timer::TimerInt)
				{
					//update and get encoder's count
					car->UpdateAllEncoders();
					ec0 =  l_m_setpoint-car->GetEncoderCount(0);
					ec1 =  r_m_setpoint-car->GetEncoderCount(1);

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


	//breathing led- indicate if program hang or not
	looper.Repeat(199, std::bind(&libsc::Led::Switch, &car->GetLed(0)), Looper::RepeatMode::kLoose);

	//Send data to grapher
	looper.Repeat(31, std::bind(&MyVarManager::sendWatchData, &m_peter), Looper::RepeatMode::kLoose);

	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> stop =
			[&](const Timer::TimerInt, const Timer::TimerInt)
	{

		static bool is_startup = true;
		const Timer::TimerInt time = System::Time();
		if (is_startup && Timer::TimeDiff(time, m_start) > 2000)
		{
			is_startup = false;
		}
		//if ADC reading = 0, light is detected
		// periodically on and off GPO

		//if GPO is off, turn it on
		if(!car->GetGpo().Get()){
			car->GetGpo().Set(1);
			gpo = 1;
			prev_adc = 2.0f;
			car->GetBuzzer().SetBeep(false);
			//prev_adc = car->GetAdc().GetResultF();
		}
		//if GPO is on
		else{
			//if ADC from 0 to 2, stop!
			if(!is_startup && (car->GetAdc().GetResultF()-prev_adc)>1.0f){
				t = false;
				car->GetBuzzer().SetBeep(false);
			}
			//turn GPO off, record prev ADC reading (normal)
			else if(car->GetAdc().GetResultF()>1.0f){
				car->GetGpo().Set(0);
				gpo = 0;
				prev_adc = 2.0f;
				car->GetBuzzer().SetBeep(false);
			}
			//turn GPO off, record prev ADC reading (I saw IR!)
			else if(car->GetAdc().GetResultF()<1.0f){
				car->GetBuzzer().SetBeep(true);
				prev_adc = 0.0f;
				if(!is_startup){
					sd_setpoint = 1000;
				}
			}
		}

	};
	looper.Repeat(250,stop,Looper::RepeatMode::kLoose);

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

			if(imageProcess.getState()==BLACK_GUIDE){
				car->GetBuzzer().SetBeep(true);
			}
			else{
				car->GetBuzzer().SetBeep(false);
			}

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
bool RunTestApp::PeggyListener(const std::vector<Byte> &bytes)
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

	return true;
}


}
