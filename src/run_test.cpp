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

#include <libsc/k60/lcd_typewriter.h>
#include <libsc/k60/led.h>
#include <libsc/k60/ov7725.h>
#include <libsc/k60/st7735r.h>
#include <libsc/k60/system.h>
#include <libsc/k60/timer.h>
#include <libutil/looper.h>
#include <libutil/misc.h>
#include <libutil/string.h>
#include "libutil/misc.h"

#include "car.h"
#include "system_res.h"
#include "run_test.h"

#define PI 3.14159265
#define ROUND_2_INT(f) ((int)(f >= 0.0 ? (f + 0.5) : (f - 0.5)))
#define FACTOR 55

using namespace libsc::k60;
using namespace libutil;
using namespace std;

namespace camera
{

void RunTestApp::Run()
{
	printf("CarTestApp\n");

	Car *car = GetSystemRes()->car;
	car->GetLcd().Clear(libutil::GetRgb565(0x33, 0xB5, 0xE5));

	const Uint image_size = car->GetCameraW() * car->GetCameraH() / 8;
	unique_ptr<Byte[]> image2(new Byte[image_size]);
	Byte* image_ana = new Byte[image_size];

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
	writer.WriteString("X_AVG:");

	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> x_avg =
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
		car->GetLcd().SetRegion({0, 144, St7735r::GetW(),
			LcdTypewriter::GetFontH()});
		writer.WriteString(String::Format("%ld\n",
				Analyze(image_ana)/FACTOR+28).c_str());
		looper.RunAfter(request, x_avg);
			};
	looper.RunAfter(150, x_avg);

	//car->SetMotorPower(0,200);
	//car->SetMotorPower(1,200);
	car->GetCamera().Start();
	looper.ResetTiming();
	while (!car->GetButton(1).IsDown())
	{
		if (car->GetCamera().IsAvailable())
		{
			memcpy(image2.get(), car->GetCamera().LockBuffer(), image_size);
			memcpy(image_ana,car->GetCamera().LockBuffer(), image_size);
			car->GetCamera().UnlockBuffer();

			car->GetLcd().SetRegion({0, 0, car->GetCameraW(), car->GetCameraH()});
			car->GetLcd().FillBits(0, 0xFFFF, image2.get(),
					car->GetCameraW() * car->GetCameraH());

			/*SERVO_MID_DEGREE + (percentage_ * SERVO_AMPLITUDE / 1000)*/
			car->SetTurning(Analyze(image_ana));
		}

		looper.Once();
	}

	car->GetCamera().Stop();
}
int16_t RunTestApp::Analyze(Byte* image){
	Car *car = GetSystemRes()->car;
	bool bitmap[60][80];

	std::vector<std::pair<int16_t, int16_t>> l_margin;
	std::vector<std::pair<int16_t, int16_t>> r_margin;

	l_margin.clear();
	r_margin.clear();

	for(int16_t image_row=0; image_row<60; image_row++){
		for(int16_t byte=0; byte<10; byte++){
			Byte check_image = image[image_row*10+byte];
			/*to bit*/
			for(int16_t bit=7; bit>=0; bit--){
				bitmap[image_row][8*byte+bit] = check_image & 0x01;
				check_image >>= 1;
			}
		}
	}
	int16_t row=0;
	for(int16_t column=0; column<60; column++){
		bool prev = bitmap[column][row];
		for(row=1; row<80; row++){
			if(bitmap[column][row]!=prev){
				if (prev){
					l_margin.push_back(std::make_pair(column,row));
				}
				else if(!prev){
					r_margin.push_back(std::make_pair(column,row));
				}
			}
			prev = bitmap[column][row];
		}
		if(l_margin.size()<column+1)
			l_margin.push_back(std::make_pair(column,0));
		if(r_margin.size()<column+1 && bitmap[column][79]==false)
			r_margin.push_back(std::make_pair(column,79));
		else if(r_margin.size()<column+1 && bitmap[column][79]==true)
			r_margin.push_back(std::make_pair(column,0));
//		char buffer[100];
//		sprintf(buffer,"%d: %d,%d\n",column,l_margin[column].second,r_margin[column].second);
//		car->GetUart().SendStr(buffer);
	}

	std::vector<std::pair<int16_t, int16_t>>midpoint;
	for(int k=0; k<60; k++){
		midpoint.push_back(std::make_pair(k,(l_margin[k].second+r_margin[k].second)/2));
	}
	double x_sum = 0;
	double near_sum =0;
	double mid_sum =0;
	double far_sum =0;

	//int16_t x_count = 0;
	int16_t black_count = 0;

	for (int far=0; far<midpoint.size()/3; far++){
		if(midpoint[far].second==0){
			black_count++;
		}
		far_sum += midpoint[far].second;
	}
	x_sum += far_sum; //*0.2;

	for (int mid=midpoint.size()/3; mid<midpoint.size()/3*2; mid++){
		if(midpoint[mid].second==0){
			black_count++;
		}
		mid_sum += midpoint[mid].second;
	}
	x_sum += mid_sum; //*0.7;

	for (int near=midpoint.size()/3*2; near<midpoint.size(); near++){
		if(midpoint[near].second==0){
			black_count++;
		}
		near_sum += midpoint[near].second;
	}
	x_sum += near_sum; //*0.1;

//	char mp_buffer[100];
//	sprintf(mp_buffer,"x_avg: %f\n", x_sum/60);
//	car->GetUart().SendStr(mp_buffer);

//	if (car->GetServo().GetDegree()-(950+(x_sum/60-28)*FACTOR*370/1000)>300 ||car->GetServo().GetDegree()-(950+(x_sum/60-28)*FACTOR*370/1000)<-300){
//		return 0;
//	}

	return -(x_sum/60-28)*FACTOR; //26
}

}
