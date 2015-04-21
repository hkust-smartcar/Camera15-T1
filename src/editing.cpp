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
#include "editing.h"
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
  //  speed(0,0,0),
  servoControl(SERVO_SETPOINT,0,0,0),
  l_kp(0.01f),
  l_ki(0.0f),
  l_kd(0.0f),
  reference(10000.0f),
//  l_speedControl(ENCODER_SETPOINT,0,0,0),
//  r_speedControl(ENCODER_SETPOINT,0,0,0),
  l_speedControl(&reference, &l_kp, &l_ki, &l_kd, 0, 600),
  m_peter()
{
	//	servoControl.SetILimit(0);
	//	skp = servoControl.GetKp();
	//	skd = servoControl.GetKd();

	//	skp = speedControl.GetKp();
	//	skd = speedControl.GetKd();

//	*l_kp = l_speedControl.GetKp();
//	r_kp = r_speedControl.GetKp();

//	*l_ki = l_speedControl.GetKi();
//	r_ki = r_speedControl.GetKi();

	ec1 = 0;
//	ec2 = 0;

	l_result = 0;
//	r_result = 0;
	//	serror = 0;

	m_instance = this;
	//	m_peter.addWatchedVar(&encoder_count, "asda");
	//	m_peter.addWatchedVar(&skp,"skp");
	//	m_peter.addWatchedVar(&skd,"skd");

	m_peter.addWatchedVar(&l_kp,"lkp");
//	m_peter.addWatchedVar(&r_kp,"rkp");

	m_peter.addWatchedVar(&l_ki,"lki");
//	m_peter.addWatchedVar(&r_ki,"rki");

	m_peter.addWatchedVar(&ec1,"ec1");
//	m_peter.addWatchedVar(&ec2, "ec2");

	m_peter.addWatchedVar(&reference,"sp");

//	m_peter.addWatchedVar(&l_result,"l_result");
//	m_peter.addWatchedVar(&r_result,"r_result");

	//	m_peter.addWatchedVar(&serror, "serror");
	//	m_peter.addWatchedVar(&cal_result, "cal_result");
	//	m_peter.Init(&GetSystemRes()->car->GetUart(), &PeggyListener);
	m_peter.Init(&PeggyListener);
}

void RunTestApp::Run()
{
	printf("CarTestApp\n");

	Car *car = GetSystemRes()->car;
	car->GetLcd().Clear(libutil::GetRgb565(0x33, 0xB5, 0xE5));

	const Uint image_size = car->GetCameraW() * car->GetCameraH() / 8;
	unique_ptr<Byte[]> image2(new Byte[image_size]);

	Looper looper;

	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> blink =
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
		car->GetLed(0).Switch();
		looper.RunAfter(request, blink);
			};
	looper.RunAfter(199, blink);

	LcdTypewriter::Config writer_conf;
	writer_conf.lcd = &car->GetLcd();
	writer_conf.bg_color = libutil::GetRgb565(0x33, 0xB5, 0xE5);
	LcdTypewriter writer(writer_conf);

//	car->GetLcd().SetRegion({0, 64, St7735r::GetW(), LcdTypewriter::GetFontH()});
//	writer.WriteString("Encoder:");
//
	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> encoder =
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
		car->UpdateAllEncoders();
//		car->GetLcd().SetRegion({0, 80, St7735r::GetW(),
//			LcdTypewriter::GetFontH()});
//		writer.WriteString(String::Format("%ld, %ld\n",
//				car->GetEncoderCount(0), car->GetEncoderCount(1)).c_str());

		ec1 =  car->GetEncoderCount(0);
//		ec2 =  (car->GetEncoderCount(1))/100;

//		l_result = l_speedControl.Calc(car->GetEncoderCount(0));
//		r_result = r_speedControl.Calc(car->GetEncoderCount(1));
		//result = speedControl.Calc(car->GetEncoderCount(0));

//		car->SetMotorPower(0, (int16_t)l_speedControl.updatePID((float)car->GetEncoderCount(0)));
//		car->SetMotorPower(0, 250);

		looper.RunAfter(request, encoder);
			};
	looper.RunAfter(89, encoder);
//
//	car->GetLcd().SetRegion({0, 96, St7735r::GetW(), LcdTypewriter::GetFontH()});
//	writer.WriteString("Servo:");
//
//	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> servo =
//			[&](const Timer::TimerInt request, const Timer::TimerInt)
//			{
//		car->GetLcd().SetRegion({0, 112, St7735r::GetW(),
//			LcdTypewriter::GetFontH()});
//		writer.WriteString(String::Format("%ld\n",
//				car->GetServo().GetDegree()).c_str());
//		looper.RunAfter(request, servo);
//			};
//	looper.RunAfter(197, servo);
//
//	car->GetLcd().SetRegion({0, 128, St7735r::GetW(), LcdTypewriter::GetFontH()});
//	writer.WriteString("error");
//
//	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> x_avg =
//			[&](const Timer::TimerInt request, const Timer::TimerInt)
//			{
//		car->GetLcd().SetRegion({0, 144, St7735r::GetW(),
//			LcdTypewriter::GetFontH()});
//		writer.WriteString(String::Format("%ld\n",
//				imageProcess.Analyze()/imageProcess.FACTOR).c_str());
//		looper.RunAfter(request, x_avg);
//			};
//	looper.RunAfter(149, x_avg);

	bool triggered = false;
	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> trigger =
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
		if (!triggered)
		{
			if(t){
				//				car->SetMotorPower(0,setpower);
				//				car->SetMotorPower(1,setpower);
				//				car->SetMotorPower(0,speed.speedCal(car,setpower));
				//				car->SetMotorPower(1,speed.speedCal(car,setpower));
				car->SetMotorPower(0,200);
				car->SetMotorPower(1,200);

				triggered=true;
				looper.RunAfter(2200, trigger);
			}
			else
			{
				looper.RunAfter(503, trigger);
			}
		}
		else if (triggered){
			car->SetMotorPower(0,0);
			car->SetMotorPower(1,0);
			triggered=false;
			t = false;
			looper.RunAfter(1000, trigger);
		}
			};
	looper.RunAfter(499, trigger);

	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> sendWatchData =
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
		m_peter.sendWatchData();
		looper.RunAfter(request, sendWatchData);
			};
	looper.RunAfter(31, sendWatchData);


	car->GetCamera().Start();
	looper.ResetTiming();
	while (!car->GetButton(1).IsDown())
	{
		if (car->GetCamera().IsAvailable())
		{
			memcpy(image2.get(), car->GetCamera().LockBuffer(), image_size);
			car->GetCamera().UnlockBuffer();

			//						car->SetMotorPower(0,setpower);
			//						car->SetMotorPower(1,setpower);

			imageProcess.start(image2.get());
			//			serror = imageProcess.Analyze();

			//			cal_result = servoControl.Calc(serror);
			//			car->SetTurning(cal_result*10);


			if (imageProcess.crossroad && abs(car->GetServo().GetDegree()-9500)<10){
				car->SetTurning(0);
			}

			printMidpoint();
			printMargin();

			//			char buffer[100];
			//			sprintf(buffer,"MIDPOINT at %d\n",imageProcess.MIDPOINT);
			//			car->GetUart().SendStr(buffer);

			char info[100]; //Q: %d\nCross: %d\n , (int)imageProcess.Q, (int)imageProcess.crossroad
			sprintf(info, "FACTOR: %d\nPOWER: %d\n", imageProcess.FACTOR, setpower);
			//			car->GetUart().SendStr(info);

		}
		looper.Once();
	}
	car->GetCamera().Stop();
}

RunTestApp &getInstance(void)
{
	return *m_instance;
}

void RunTestApp::PeggyListener(const std::vector<Byte> &bytes)
{
	switch (bytes[0])
	{
	case 'a':
		m_instance->imageProcess.MIDPOINT++; break;

	case 'b':
		m_instance->imageProcess.MIDPOINT--; break;

	case 'v':
		m_instance->t = true; break;

	case 'f':
//		m_instance->GetSystemRes()->car->SetMotorPower(0,m_instance->);
		m_instance->GetSystemRes()->car->SetMotorPower(1,m_instance->setpower);
		//		m_instance->GetSystemRes()->car->SetMotorPower(0,350);
		//		m_instance->GetSystemRes()->car->SetMotorPower(1,350);
		break;

	case 'g':
		m_instance->GetSystemRes()->car->SetMotorPower(0,0);
		m_instance->GetSystemRes()->car->SetMotorPower(1,0);
		break;

	case 'p':
		//		m_instance->servoControl.SetKp(m_instance->servoControl.GetKp()+0.01f);
		//		m_instance->skp = m_instance->servoControl.GetKp();

//		m_instance->l_speedControl.SetKp(m_instance->l_speedControl.GetKp()+0.0001f);
//		m_instance->l_kp = m_instance->l_speedControl.GetKp();

		m_instance->l_kp += 0.001f;

//		m_instance->GetSystemRes()->car->SetMotorPower(0,m_instance->GetSystemRes()->car->GetMotor(0).GetPower()+m_instance->l_speedControl.updatePID_ori(m_instance->GetSystemRes()->car->GetEncoderCount(0)));

		break;

	case 'P':
		//		m_instance->servoControl.SetKp(m_instance->servoControl.GetKp()-0.01f);
		//		m_instance->skp = m_instance->servoControl.GetKp();

//		m_instance->l_speedControl.SetKp(m_instance->l_speedControl.GetKp()-0.0001f);
//		m_instance->l_kp = m_instance->l_speedControl.GetKp();

		m_instance->l_kp -= 0.001f;

//		m_instance->GetSystemRes()->car->SetMotorPower(0,m_instance->GetSystemRes()->car->GetMotor(0).GetPower()+m_instance->l_speedControl.updatePID_ori(m_instance->GetSystemRes()->car->GetEncoderCount(0)));

		break;

	case 'r':
		m_instance->reference += 100.0f;
		break;

	case 'R':
		m_instance->reference -= 100.0f;
		break;

//	case 'q':
//
//		m_instance->r_speedControl.SetKp(m_instance->r_speedControl.GetKp()+0.0001f);
//		m_instance->r_kp = m_instance->r_speedControl.GetKp();
//
//		m_instance->GetSystemRes()->car->SetMotorPower(1,m_instance->GetSystemRes()->car->GetMotor(1).GetPower()+m_instance->r_speedControl.Calc(m_instance->GetSystemRes()->car->GetEncoderCount(1)));
//
//		break;
//
//	case 'Q':
//
//		m_instance->r_speedControl.SetKp(m_instance->r_speedControl.GetKp()-0.0001f);
//		m_instance->r_kp = m_instance->r_speedControl.GetKp();
//
//		m_instance->GetSystemRes()->car->SetMotorPower(1,m_instance->GetSystemRes()->car->GetMotor(1).GetPower()+m_instance->r_speedControl.Calc(m_instance->GetSystemRes()->car->GetEncoderCount(1)));
//
//		break;


	case 'i':
		//		m_instance->servoControl.SetKd(m_instance->servoControl.GetKd()+0.01f);
		//		m_instance->skd = m_instance->servoControl.GetKd();

//		m_instance->l_speedControl.SetKi(m_instance->l_speedControl.GetKi()+0.0001f);
//		m_instance->l_ki = m_instance->l_speedControl.GetKi();

		m_instance->l_ki++;

//		m_instance->GetSystemRes()->car->SetMotorPower(0,m_instance->GetSystemRes()->car->GetMotor(0).GetPower()+m_instance->l_speedControl.updatePID_ori(m_instance->GetSystemRes()->car->GetEncoderCount(0)));

		break;

	case 'I':
//		m_instance->l_speedControl.SetKi(m_instance->l_speedControl.GetKi()-0.0001f);
//		m_instance->l_ki = m_instance->l_speedControl.GetKi();

		m_instance->l_ki--;

//		m_instance->GetSystemRes()->car->SetMotorPower(0,m_instance->GetSystemRes()->car->GetMotor(0).GetPower()+m_instance->l_speedControl.updatePID_ori(m_instance->GetSystemRes()->car->GetEncoderCount(0)));

		break;

//	case 'j':
//		//		m_instance->servoControl.SetKd(m_instance->servoControl.GetKd()+0.01f);
//		//		m_instance->skd = m_instance->servoControl.GetKd();
//
//		m_instance->r_speedControl.SetKi(m_instance->r_speedControl.GetKi()+0.0001f);
//		m_instance->r_ki = m_instance->r_speedControl.GetKi();
//
//		m_instance->GetSystemRes()->car->SetMotorPower(1,m_instance->GetSystemRes()->car->GetMotor(1).GetPower()+m_instance->r_speedControl.Calc(m_instance->GetSystemRes()->car->GetEncoderCount(1)));
//
//		break;
//
//	case 'J':
//		m_instance->r_speedControl.SetKi(m_instance->r_speedControl.GetKi()-0.0001f);
//		m_instance->r_ki = m_instance->r_speedControl.GetKi();
//
//		m_instance->GetSystemRes()->car->SetMotorPower(1,m_instance->GetSystemRes()->car->GetMotor(1).GetPower()+m_instance->r_speedControl.Calc(m_instance->GetSystemRes()->car->GetEncoderCount(1)));
//
//		break;



		//
		//	case 'e':
		//		//		m_instance->servoControl.SetKd(m_instance->servoControl.GetKd()-0.01f);
		//		//		m_instance->skd = m_instance->servoControl.GetKd();
		//
		//		m_instance->speedControl.SetKd(m_instance->speedControl.GetKd()-0.0001f);
		//		m_instance->skd = m_instance->speedControl.GetKd();
		//
		//		m_instance->GetSystemRes()->car->SetMotorPower(0,m_instance->GetSystemRes()->car->GetMotor(0).GetPower()+m_instance->speedControl.Calc(m_instance->GetSystemRes()->car->GetEncoderCount(0)));
		//		m_instance->GetSystemRes()->car->SetMotorPower(1,m_instance->GetSystemRes()->car->GetMotor(1).GetPower()+m_instance->speedControl.Calc(m_instance->GetSystemRes()->car->GetEncoderCount(1)));
		//
		//		break;

	case 's':
		m_instance->setpower+=10; break;

	case 't':
		m_instance->setpower-=10; break;

	case 'y':
		m_instance->GetSystemRes()->car->SetMotorPower(0,-180);
		m_instance->GetSystemRes()->car->SetMotorPower(1,-180);
		break;

	}
}

void RunTestApp::printMidpoint(){
	Car *car = GetSystemRes()->car;

	bool* array_ptr;
	for (int i=0; i<car->GetCameraH();i++){
		array_ptr= imageProcess.bitmap[i];
		car->GetLcd().SetRegion(libsc::Lcd::Rect(0,i,car->GetCameraW(),1));
		car->GetLcd().FillBits(0,0xFFFF,array_ptr,car->GetCameraW());
	}

	for(Uint i=0; i<car->GetCameraH(); i++){
		car->GetLcd().SetRegion({imageProcess.MIDPOINT, i, 1, 1});
		car->GetLcd().FillColor(St7735r::kCyan);
	}

	for(Uint i=30; i<40; i++){
		car->GetLcd().SetRegion({imageProcess.midpoint[i], i, 1, 1});
		car->GetLcd().FillColor(St7735r::kRed);
	}

}

void RunTestApp::printMargin(){
	Car *car = GetSystemRes()->car;

	for(Uint i=0; i<car->GetCameraH(); i++){
		car->GetLcd().SetRegion({imageProcess.margin[i][0], i, 1, 1});
		car->GetLcd().FillColor(St7735r::kBlue);
		car->GetLcd().SetRegion({imageProcess.margin[i][1], i, 1, 1});
		car->GetLcd().FillColor(St7735r::kBlue);
	}
}

}
