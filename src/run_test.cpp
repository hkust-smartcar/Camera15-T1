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

RunTestApp::RunTestApp(SystemRes *res, uint16_t motor_setpoint, float skp, float ski, float skd, float s_skp, float s_skd)
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

  show_error(0.0),

  RA_time(System::Time()),

  servo_Control(&s_setpoint, &s_kp, &s_ki, &s_kd,&s_kp_straight, &s_ki_straight, &s_kd_straight, -1000, 1000),
  l_speedControl(&l_m_setpoint, &l_kp, &l_ki, &l_kd,0,0,0, 0, 950),
  r_speedControl(&r_m_setpoint, &r_kp, &r_ki, &r_kd,0,0,0, 0, 950),

  imageProcess(GetSystemRes()->car),

  m_start(0),
  m_is_stop(false),

  prev_adc(0),
  gpo(0),

  prev_time(System::Time()),
  time_difference(0)

{
	//raw data: left & right encoder, servo angle
	ec0 = 0;
	ec1 = 0;

	l_result=0;
	r_result=0;

	printLKp = 0;
	printLKi = 0;
	printLKd = 0;

	printRKp = 0;
	printRKi = 0;
	printRKd = 0;

	m_instance = this;

#ifdef PGRAPHER

	//for grapher use
	m_peter.addWatchedVar(&sd_setpoint, "sd_setpoint");
	m_peter.addWatchedVar(&l_result,"left pwm");
	m_peter.addWatchedVar(&r_result,"right pwm");
	m_peter.addWatchedVar(&printLKp, "*LKP");
	m_peter.addWatchedVar(&printLKi, "*LKI");
	m_peter.addWatchedVar(&printLKd, "*LKD");


	m_peter.addSharedVar(&s_kp,"skp");
//	m_peter.addSharedVar(&s_ki,"ski");
	m_peter.addSharedVar(&s_kd,"skd");

	m_peter.Init(&PeggyListener);
	prev_adc = GetSystemRes()->car->GetAdc().GetResultF();

#endif

}

void RunTestApp::DetectEmergencyStop(){

	//Get car
	Car *car = GetSystemRes()->car;

	static bool is_startup = true;
	const Timer::TimerInt time = System::Time();
	if (is_startup && Timer::TimeDiff(time, m_start) > 3000)
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

void RunTestApp::Run()
{

	time_difference = System::Time() - prev_time;
	prev_time = System::Time();

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

#ifdef PGRAPHER
	/*Send data to grapher*/
	looper.Repeat(31, std::bind(&MyVarManager::sendWatchData, &m_peter), Looper::RepeatMode::kLoose);

#endif

#ifdef CAR_WITH_BT
	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> bt =
			[&](const Timer::TimerInt, const Timer::TimerInt)
	{
//		char received;
//		if(car->GetUart().PeekChar(&received)){
//			switch(received){
//
//			//move & stop
//			case 'f':
//				//reset pid
//				l_speedControl.reset();
//				r_speedControl.reset();
//				servo_Control.reset();
//
//				if(motor_run)
//					motor_run = false;
//				else{
//					m_start = System::Time();
//					m_is_stop = false;
//					motor_run = true;
//				}
//				break;
//
//			//faster!
//			case 'r':
//				l_m_setpoint += 100.0f;
//				r_m_setpoint += 100.0f;
//				sd_setpoint += 100.0f;
//				break;
//
//			//slower!
//			case 'R':
//				l_m_setpoint -= 100.0f;
//				r_m_setpoint -= 100.0f;
//				sd_setpoint -= 100.0f;
//				break;
//
//			//servo kp
//			case 'p':
//				s_kp += 0.005f;
//				break;
//
//			case 'P':
//				s_kp -= 0.005f;
//				break;
//
//			//servo kd
//			case 'd':
//				s_kd += 0.0005f;
//				break;
//
//			case 'D':
//				s_kd -= 0.0005f;
//				break;
//
//			}
//		}

		char buffer[100];
//		sprintf(buffer,"%f    %f    %d    %ld    %ld    %ld    %ld\n", s_kp,s_kd, sd_setpoint, l_result, r_result,ec0,ec1);
		sprintf(buffer, "%ld    %ld    %ld\n",time_difference,ec0,ec1);
//		if(servo_Control.returnTypeOfPID() == STRAIGHT){
//			sprintf(buffer,"STRAIGHT:	%f\n", show_error);
//		}
//		else if(servo_Control.returnTypeOfPID() == TURNING){
//			sprintf(buffer,"TURNING:	%f\n", show_error);
//		}
		car->GetUart().SendStr(buffer);
	};
	looper.Repeat(31, bt, Looper::RepeatMode::kPrecise);
#endif

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
//			car->GetBuzzer().SetBeep(false);
			//prev_adc = car->GetAdc().GetResultF();
		}
		//if GPO is on
		else{
			//if ADC from 0 to 2, stop!
			if(!is_startup && (car->GetAdc().GetResultF()-prev_adc)>1.0f){
				motor_run = false;
//				car->GetBuzzer().SetBeep(false);
			}
			//turn GPO off, record prev ADC reading (normal)
			else if(car->GetAdc().GetResultF()>1.0f){
				car->GetGpo().Set(0);
				gpo = 0;
				prev_adc = 2.0f;
//				car->GetBuzzer().SetBeep(false);
			}
			//turn GPO off, record prev ADC reading (I saw IR!)
			else if(car->GetAdc().GetResultF()<1.0f){
//				car->GetBuzzer().SetBeep(true);
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

		show_error = error;

		//set angle with servo PID controller and image process result
		//negative for correcting direction
		car->SetTurning((int16_t)servo_Control.updatePID_ori(-error));

		//software differential
		l_m_setpoint =  (float)software_differential.turn_left_encoder(sd_setpoint,car->GetServo().GetDegree(),SERVO_MID_DEGREE,3700);
		r_m_setpoint = (float) software_differential.turn_right_encoder(sd_setpoint,car->GetServo().GetDegree(),SERVO_MID_DEGREE,3700);

		//update and get encoder's count
		car->UpdateAllEncoders();
		ec0 =  car->GetEncoderCount(0);
		ec1 =  car->GetEncoderCount(1);

		//if motor_run is true, update PID and give power to car, else stop the car
		if(motor_run && !m_is_stop){

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

		time_difference = System::Time()-prev_time;
		prev_time = System::Time();
	};
	looper.Repeat(11, servo, Looper::RepeatMode::kPrecise);


	/*Get image*/
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

#ifdef PGRAPHER

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

		if(m_instance->motor_run)
			m_instance->motor_run = false;
		else{

			m_instance->m_start = System::Time();
			m_instance->m_is_stop = false;
			m_instance->motor_run = true;
		}
		break;

	//faster!
	case 'r':
		m_instance->l_m_setpoint += 25.0f;
		m_instance->r_m_setpoint += 25.0f;
		m_instance->sd_setpoint += 25.0f;
		break;

	//slower!
	case 'R':
		m_instance->l_m_setpoint -= 25.0f;
		m_instance->r_m_setpoint -= 25.0f;
		m_instance->sd_setpoint -= 25.0f;
		break;
	}

	return true;
}
#endif

}
