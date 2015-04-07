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
  imageProcess()
{
	black_count = 0;
	m_instance = this;

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
	looper.RunAfter(200, blink);

	LcdTypewriter::Config writer_conf;
	writer_conf.lcd = &car->GetLcd();
	writer_conf.bg_color = libutil::GetRgb565(0x33, 0xB5, 0xE5);
	LcdTypewriter writer(writer_conf);

	car->GetLcd().SetRegion({0, 64, St7735r::GetW(), LcdTypewriter::GetFontH()});
	writer.WriteString("Encoder:");

	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> encoder =
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
		car->UpdateAllEncoders();
		car->GetLcd().SetRegion({0, 80, St7735r::GetW(),
			LcdTypewriter::GetFontH()});
		writer.WriteString(String::Format("%ld, %ld\n",
				car->GetEncoderCount(0), car->GetEncoderCount(1)).c_str());
		looper.RunAfter(request, encoder);
			};
	looper.RunAfter(250, encoder);

	car->GetLcd().SetRegion({0, 96, St7735r::GetW(), LcdTypewriter::GetFontH()});
	writer.WriteString("Servo:");

	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> servo =
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
		car->GetLcd().SetRegion({0, 112, St7735r::GetW(),
			LcdTypewriter::GetFontH()});
		writer.WriteString(String::Format("%ld\n",
				car->GetServo().GetDegree()).c_str());
		looper.RunAfter(request, servo);
			};
	looper.RunAfter(200, servo);

	car->GetLcd().SetRegion({0, 128, St7735r::GetW(), LcdTypewriter::GetFontH()});
	writer.WriteString("error");

	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> x_avg =
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
		car->GetLcd().SetRegion({0, 144, St7735r::GetW(),
			LcdTypewriter::GetFontH()});
		writer.WriteString(String::Format("%ld\n",
				Analyze()/FACTOR).c_str());
		looper.RunAfter(request, x_avg);
			};
	looper.RunAfter(150, x_avg);

	bool triggered = false;
	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> trigger =
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
		if (!triggered)
		{
			if(t){
				car->SetMotorPower(0,setpower);
				car->SetMotorPower(1,setpower);
				triggered=true;
				looper.RunAfter(2000, trigger);
			}
			else
			{
				looper.RunAfter(500, trigger);
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
	looper.RunAfter(500, trigger);

	car->GetCamera().Start();
	looper.ResetTiming();
	while (!car->GetButton(1).IsDown())
	{
		if (car->GetCamera().IsAvailable())
		{
			memcpy(image2.get(), car->GetCamera().LockBuffer(), image_size);
			car->GetCamera().UnlockBuffer();

			//			car->SetMotorPower(0,setpower);
			//			car->SetMotorPower(1,setpower);

			imageProcess.start(image2.get(), &car->GetUart());

			car->SetTurning(Analyze());

			printMidpoint();
			printMargin();

			char received;
			if(car->GetUart().PeekChar(&received)){
				if(received == 'a')
					imageProcess.MIDPOINT++;
				else if (received == 'b')
					imageProcess.MIDPOINT--;
				else if(received == 'e')
					t = true;
				else if(received == 'f'){
					car->SetMotorPower(0,setpower);
					car->SetMotorPower(1,setpower);
				}
				else if (received == 'g'){
					car->SetMotorPower(0,0);
					car->SetMotorPower(1,0);
				}
				else if (received == 'i'){
					FACTOR+=5;
				}
				else if (received == 'j'){
					FACTOR-=5;
				}
				else if (received == 's'){
					setpower+=10;
				}
				else if (received == 't'){
					setpower-=10;
				}
			}


		}
		looper.Once();
	}
	car->GetCamera().Stop();
}

double MultiplyRatio(double err, int FACTOR){

	if(err>15 || err<-15){
		return err*(FACTOR*1.3);
	}
	else if (err>10 || err <-10){
		return err*(FACTOR*0.7);
	}
	else{
		return err*FACTOR;
	}
}

int RunTestApp::Analyze(void){

	double error = imageProcess.MIDPOINT - MidpointSumCal(30,40)/10;

	return MultiplyRatio(error, FACTOR);
}

double RunTestApp::MidpointSumCal(int start, int end){
	Car *car = GetSystemRes()->car;
	double sum = 0;

	for(int k=start; k<end; k++){
		black_count = 0;

		if (imageProcess.midpoint[k]==0)
			black_count++;

		if (imageProcess.margin[k][0] == 5 && imageProcess.margin[k][1] == car->GetCameraW()-5){
			return 10*(imageProcess.midpoint[k]=(imageProcess.midpoint[k+5]+imageProcess.midpoint[k+4]+imageProcess.midpoint[k+3]+imageProcess.midpoint[k+2]+imageProcess.midpoint[k+1])/5);
		}

		sum += imageProcess.midpoint[k];
	}

	return sum;
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

	for(Uint i=0; i<60; i++){
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
