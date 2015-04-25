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

#include <libsc/lcd_typewriter.h>
#include <libsc/k60/led.h>
#include <libsc/k60/ov7725.h>
#include <libsc/st7735r.h>
#include <libsc/system.h>
#include <libsc/timer.h>
#include <libutil/looper.h>
#include <libutil/misc.h>
#include <libutil/string.h>
#include "libutil/misc.h"
#include "libsc/joystick.h"

#include "car.h"
#include "system_res.h"
#include "run_test.h"
#include "ImageProcess.h"

using namespace libsc;
using namespace libsc::k60;
using namespace libutil;
using namespace std;

namespace camera
{

RunTestApp *m_instance;

RunTestApp::RunTestApp(SystemRes *res)
: App(res),
  imageProcess(),
  l_kp(0.0125f),
  l_ki(0.00000000000001f),
  l_kd(0.0f),

  r_kp(0.0099f),
  r_ki(0.00000000000001421f),
  r_kd(0.0f),

  s_kp(0.93f),
  s_ki(0.0f),
  s_kd(0.0f),

  l_m_setpoint(10300.0f),
  r_m_setpoint(10300.0f),

  sd_setpoint(10300),

  s_setpoint(0.0f),

  l_speedControl(&l_m_setpoint, &l_kp, &l_ki, &l_kd, 0, 600),
  r_speedControl(&r_m_setpoint, &r_kp, &r_ki, &r_kd, 0, 600),
  servo_Control(&s_setpoint, &s_kp, &s_ki, &s_kd, -1000, 1000),

  m_peter()
{
	//raw data: left & right encoder, servo angle
	ec0 = 0;
	ec1 = 0;
	s_degree = 0;

	l_result = 0;
	r_result = 0;
	s_result = 0;

	m_instance = this;

	//for grapher use

	m_peter.addWatchedVar(&s_kp,"skp");
	m_peter.addWatchedVar(&s_ki,"ski");
	m_peter.addWatchedVar(&s_kd,"skd");
	m_peter.addWatchedVar(&s_degree, "serror");
	m_peter.addWatchedVar(&s_result,"sresult");
	m_peter.addWatchedVar(&sd_setpoint,"sp");

	//

	/* left wheel

	m_peter.addWatchedVar(&ec1,"ec1");
	m_peter.addWatchedVar(&r_kp,"rkp");
	m_peter.addWatchedVar(&r_ki,"rki");
	m_peter.addWatchedVar(&r_kd,"rkd");

	 */

	/* right wheel

	m_peter.addWatchedVar(&ec2,"ec2");
	m_peter.addWatchedVar(&l_kp,"lkp");
	m_peter.addWatchedVar(&l_ki,"lki");
	m_peter.addWatchedVar(&l_kd,"lkd");

	*/

	m_peter.Init(&PeggyListener);
}

void RunTestApp::Run()
{
	printf("CarTestApp\n");

	//Get car
	Car *car = GetSystemRes()->car;

	//Get LCD
	car->GetLcd().Clear(libutil::GetRgb565(0x33, 0xB5, 0xE5));

	//Get image size (80*60)/byte_size
	const Uint image_size = car->GetCameraW() * car->GetCameraH() / 8;
	unique_ptr<Byte[]> image2(new Byte[image_size]);

	Looper looper;

	//breathing led- indicate if program hang or not
	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> blink =
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
				car->GetLed(0).Switch();
				looper.RunAfter(request, blink);
			};
	looper.RunAfter(199, blink);

	//Initiate LCD writer for printing real time information
		LcdTypewriter::Config writer_conf;
		writer_conf.lcd = &car->GetLcd();
		writer_conf.bg_color = libutil::GetRgb565(0x33, 0xB5, 0xE5);
		LcdTypewriter writer(writer_conf);

	//Print Servo degree
		car->GetLcd().SetRegion({0, 64, St7735r::GetW(), LcdTypewriter::GetFontH()});
		writer.WriteString("Encoder:");

	//Update encoder count, input to and update speed PID controller
	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> encoder =
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
				//update and get encoder's count
				car->UpdateAllEncoders();
				ec0 =  car->GetEncoderCount(0);
				ec1 =  car->GetEncoderCount(1);

				//print encoder count
				car->GetLcd().SetRegion({0, 80, St7735r::GetW(),
					LcdTypewriter::GetFontH()});
				writer.WriteString(String::Format("%ld, %ld\n",
						ec0, ec1).c_str());

				//if t is true, update PID and give power to car, else stop the car
				if(t){
					l_result = (int32_t)l_speedControl.updatePID((float)car->GetEncoderCount(0));
					car->SetMotorPower(0,l_result);

					r_result = (int32_t)r_speedControl.updatePID((float)car->GetEncoderCount(1));
					car->SetMotorPower(1,r_result);
				}
				else{
					car->SetMotorPower(0,0);
					car->SetMotorPower(1,0);
				}

				looper.RunAfter(request, encoder);
			};
	looper.RunAfter(89, encoder);

	//Print Servo degree
	car->GetLcd().SetRegion({0, 96, St7735r::GetW(), LcdTypewriter::GetFontH()});
	writer.WriteString("Servo:");

	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> servo =
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{

				//print info
				car->GetLcd().SetRegion({0, 112, St7735r::GetW(),
					LcdTypewriter::GetFontH()});
				writer.WriteString(String::Format("%ld\n",
						car->GetServo().GetDegree()).c_str());

				looper.RunAfter(request, servo);

			};
	looper.RunAfter(197, servo);

	//Send data to grapher
	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> sendWatchData =
			[&](const Timer::TimerInt request, const Timer::TimerInt)
	{
		m_peter.sendWatchData();
		looper.RunAfter(request, sendWatchData);
	};
	looper.RunAfter(31, sendWatchData);

	//	Timer::TimerInt check_ms = System::Time();

	//Get image
	car->GetCamera().Start();
	looper.ResetTiming();
	while (!car->GetButton(1).IsDown())
	{
		if (car->GetCamera().IsAvailable())
		{
			memcpy(image2.get(), car->GetCamera().LockBuffer(), image_size);
			car->GetCamera().UnlockBuffer();

			//start image processing
			imageProcess.start(image2.get());

			//set angle with servo PID controller and image process result
			//negative for correcting direction
			//*100 as error is too small
			s_result = (int16_t)servo_Control.updatePID_ori(-(float)imageProcess.Analyze()*100);
			car->SetTurning(s_result);

//			//set speed according to software differential
			l_m_setpoint =  (float)software_differential.turn_left_encoder(sd_setpoint,car->GetServo().GetDegree(),9500,3700);
			r_m_setpoint = (float) software_differential.turn_right_encoder(sd_setpoint,car->GetServo().GetDegree(),9500,3700);

			if (imageProcess.crossroad && abs(car->GetServo().GetDegree()-9500)<10){
				car->SetTurning(0);
			}
		}
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
		else
			m_instance->t = true;
		break;

	//for PID tunning
	case 'p':
		m_instance->s_kp += 0.01f;
//		m_instance->l_kp += 0.001f;
//		m_instance->r_kp += 0.001f;
		break;

	case 'P':
		m_instance->s_kp -= 0.01f;
//		m_instance->l_kp -= 0.001f;
//		m_instance->r_kp -= 0.001f;
		break;

	case 'i':
		m_instance->s_ki += 0.0000001f;
//		m_instance->l_ki += 0.0000001f;
//		m_instance->r_ki += 0.0000001f;
		break;

	case 'I':
		m_instance->s_ki -= 0.0000001f;
//		m_instance->l_ki -= 0.0000001f;
//		m_instance->r_ki -= 0.0000001f;
		break;

	case 'd':
		m_instance->s_kd += 0.0001f;
//		m_instance->l_kd += 0.00001f;
//		m_instance->r_kd += 0.00001f;
		break;

	case 'D':
		m_instance->s_kd -= 0.0001f;
//		m_instance->l_kd -= 0.00001f;
//		m_instance->r_kd -= 0.00001f;
		break;

	case 's':
		m_instance->s_setpoint += 1.0f;
		break;

	case 'S':
		m_instance->s_setpoint -= 1.0f;
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

	// move backward
	case 'y':
		m_instance->GetSystemRes()->car->SetMotorPower(0,-240);
		m_instance->GetSystemRes()->car->SetMotorPower(1,-240);
		break;

	}
}


}
