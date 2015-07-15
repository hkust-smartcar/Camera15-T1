/*
 * scstudio_test_app.cpp
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
#include "scstudio_test_app.h"
#include "ImageProcess.h"


#include "libbase/k60/pit.h"
#include "libbase/k60/clock_utils.h"

using namespace libsc;
using namespace libsc::k60;
using namespace libutil;
using namespace std;

namespace camera
{

SCStudioTestApp::SCStudioTestApp(SystemRes *res, uint16_t motor_setpoint, float skp, float ski, float skd, float s_skp, float s_skd)
: App(res),

  s_kp(skp),
  s_ki(ski),
  s_kd(skd),

  s_kp_straight(s_skp),
  s_ki_straight(0.0f),
  s_kd_straight(s_skd),

  s_setpoint(0.0f),

  //19ms
  l_kp(0.0108f),
  l_ki(0.0f),
  l_kd(0.00055f),

  r_kp(0.0104f),
  r_ki(0.0f),
  r_kd(0.00055f),

  //19 ms
  l_m_setpoint(motor_setpoint),
  r_m_setpoint(motor_setpoint),

  sd_setpoint((uint16_t)motor_setpoint),

  RA_time(System::Time()),

  servo_Control(&s_setpoint, &s_kp, &s_ki, &s_kd,&s_kp_straight, &s_ki_straight, &s_kd_straight, -1000, 1000),
  l_speedControl(&l_m_setpoint, &l_kp, &l_ki, &l_kd,0,0,0, 0, 950),
  r_speedControl(&r_m_setpoint, &r_kp, &r_ki, &r_kd,0,0,0, 0, 950),

  imageProcess(GetSystemRes()->car),

  m_start(0),
  m_is_stop(false),

  prev_adc(0),
  gpo(0)

{
	//raw data: left & right encoder, servo angle
	ec0 = 0;
	ec1 = 0;

	prev_adc = GetSystemRes()->car->GetAdc().GetResultF();

#ifdef CAR_WITH_BT
	scstudio.SetUart(&(GetSystemRes()->car->GetUart()));
#endif

}

void SCStudioTestApp::DetectEmergencyStop(){

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

	if (!is_startup && motor_run && (abs(count) < 30 || abs(count2) < 30))
	{
		if (m_emergency_stop_state.is_triggered)
		{
			if (Timer::TimeDiff(time, m_emergency_stop_state.trigger_time) > 20)
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

void SCStudioTestApp::Run()
{

	/************************INITIALIZATION***********************/

	//Get car
	Car *car = GetSystemRes()->car;

	//Get LCD
	car->GetLcd().Clear(libutil::GetRgb565(0x33, 0xB5, 0xE5));

	//Get image size (80*60)/byte_size
	const uint16_t image_size = car->GetCameraW() * car->GetCameraH() / 8;
	unique_ptr<Byte[]> image2(new Byte[image_size]);

	//record start time
	m_start = System::Time();

	/************************LOOPER***********************/
	Looper looper;

	/*breathing led- indicate if program hang or not*/
	looper.Repeat(199, std::bind(&libsc::Led::Switch, &car->GetLed(0)), Looper::RepeatMode::kLoose);

	/*Stop car by checking IR constantly*/
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
		}
		//if GPO is on
		else{
			//if ADC from 0 to 2, stop!
			if(!is_startup && (car->GetAdc().GetResultF()-prev_adc)>1.0f){
				motor_run = false;
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
					sd_setpoint = 700;
				}
			}
		}

	};
	looper.Repeat(250,stop,Looper::RepeatMode::kLoose);

	/*Update servo error, input to and update servo PID controller*/
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
		//imageProcess.printResult();

		if(imageProcess.getState() == BLACK_GUIDE)
		{
			car->GetBuzzer().SetBeep(true);
		}
		else
		{
			car->GetBuzzer().SetBeep(false);
		}

		show_error = error;

		//set angle with servo PID controller and image process result
		//negative for correcting direction
		car->SetTurning((int16_t)servo_Control.updatePID_ori(-error));

		//software differential
		l_m_setpoint =  (float)software_differential.turn_left_encoder(sd_setpoint,car->GetServo().GetDegree(),SERVO_MID_DEGREE,3700);
		r_m_setpoint = (float) software_differential.turn_right_encoder(sd_setpoint,car->GetServo().GetDegree(),SERVO_MID_DEGREE,3700);

		//update and get encoder's count
		car->UpdateAllEncoders();
		ec0 =  l_m_setpoint-car->GetEncoderCount(0);
		ec1 =  r_m_setpoint-car->GetEncoderCount(1);

		//if motor_run is true, update PID and give power to car, else stop the car
		if(motor_run && !m_is_stop){
			car->SetMotorPower(0,(int32_t)l_speedControl.updatePID((float)car->GetEncoderCount(0)));

			car->SetMotorPower(1,(int32_t)r_speedControl.updatePID((float)car->GetEncoderCount(1)));

			ScStudio::GraphPack pack(11);

			pack.Pack(0, sd_setpoint);
			pack.PackF(1, l_speedControl.returnKpresult());
			pack.PackF(2, l_speedControl.returnKiresult());
			pack.PackF(3, l_speedControl.returnKdresult());
			pack.Pack(4, car->GetMotor(0).GetPower());
			pack.PackF(5, (float)car->GetEncoderCount(0));

			pack.PackF(6, r_speedControl.returnKpresult());
			pack.PackF(7, r_speedControl.returnKiresult());
			pack.PackF(8, r_speedControl.returnKdresult());
			pack.Pack(9, car->GetMotor(1).GetPower());
			pack.PackF(10, (float)car->GetEncoderCount(1));

			scstudio.SendGraph(pack);

			DetectEmergencyStop();
		}
		else {
			car->SetMotorPower(0,0);
			car->SetMotorPower(1,0);
		}

	};
	looper.Repeat(11, servo, Looper::RepeatMode::kPrecise);

#ifdef CAR_WITH_BT

	//Send camera
	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> sendCam =
			[&](const Timer::TimerInt, const Timer::TimerInt)
	{
		scstudio.SendCamera(image2.get(),image_size);
	};
	looper.Repeat(50, sendCam, Looper::RepeatMode::kPrecise);

	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> bt =
			[&](const Timer::TimerInt, const Timer::TimerInt)
	{
		char received;
		if(car->GetUart().PeekChar(&received)){
			switch(received){
			//move & stop
			case 'f':
				//reset pid
				l_speedControl.reset();
				r_speedControl.reset();
				servo_Control.reset();

				if(motor_run)
					motor_run = false;
				else{

					m_start = System::Time();
					m_is_stop = false;
					motor_run = true;
				}
				break;

				//faster!
			case 'r':
				l_m_setpoint += 25.0f;
				r_m_setpoint += 25.0f;
				sd_setpoint += 25.0f;
				break;

				//slower!
			case 'R':
				l_m_setpoint -= 25.0f;
				r_m_setpoint -= 25.0f;
				sd_setpoint -= 25.0f;
				break;
			}
		}

	};
	looper.Repeat(31, bt, Looper::RepeatMode::kPrecise);

#endif

	/*Get image*/
	car->GetCamera().Start();
	looper.ResetTiming();
	while (!car->GetButton(1).IsDown())
	{
		looper.Once();
	}
	car->GetCamera().Stop();


}

}
