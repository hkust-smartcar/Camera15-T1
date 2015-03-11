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

#define FACTOR 200
#define MIDPOINT 12

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
	//Byte* image_ana = new Byte[image_size];

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
		FindMargin();
		if(Analyze()<1000){
			writer.WriteString(String::Format("%ld\n",
					Analyze()/FACTOR+MIDPOINT).c_str());
		}
		else if(Analyze()>=1000){
			writer.WriteString(String::Format("RA at: %ld\n",
					Analyze()-1000).c_str());
		}
		looper.RunAfter(request, x_avg);
			};
	looper.RunAfter(150, x_avg);

	bool no_ec_reading = car->GetEncoderCount(0)==0 && car->GetEncoderCount(1)==0;
	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> motor_safety =
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
		car->UpdateAllEncoders();
		if(no_ec_reading && (car->GetMotor(0).GetPower()>0||car->GetMotor(1).GetPower()>0)){
			car->SetMotorPower(0,0);
			car->SetMotorPower(1,0);
		}
		looper.RunAfter(request, motor_safety);
			};

	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> trigger =
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
		if (!triggered)
		{
			if (Trigger(car->GetEncoderCount(0),car->GetEncoderCount(1))){
				car->SetMotorPower(0,250);
				car->SetMotorPower(1,250);
				triggered=true;
				//				looper.RunAfter(1005, motor_safety);
				looper.RunAfter(3000, trigger);
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
			looper.RunAfter(2000, trigger);
		}
			};
	looper.RunAfter(500, trigger);

	car->SetMotorPower(0,200);
	car->SetMotorPower(1,200);
	car->GetCamera().Start();
	looper.ResetTiming();
	while (!car->GetButton(1).IsDown())
	{
		if (car->GetCamera().IsAvailable())
		{
			memcpy(image2.get(), car->GetCamera().LockBuffer(), image_size);
			//memcpy(image2, car->GetCamera().LockBuffer(), image_size);
			memcpy(image,car->GetCamera().LockBuffer(), image_size);
			//memcpy(image_ana,car->GetCamera().LockBuffer(), image_size);
			car->GetCamera().UnlockBuffer();

			car->GetLcd().SetRegion({0, 0, car->GetCameraW(), car->GetCameraH()});
			car->GetLcd().FillBits(0, 0xFFFF, image2.get(),
					car->GetCameraW() * car->GetCameraH());

			/*SERVO_MID_DEGREE + (percentage_ * SERVO_AMPLITUDE / 1000)*/
			FindMargin();
			car->SetTurning(Analyze());
		}

		looper.Once();
	}

	car->GetCamera().Stop();
}

bool RunTestApp::Trigger(int32_t l_encoder_reading, int32_t r_encoder_reading){
	if(l_encoder_reading>100||r_encoder_reading>100){
		return true;
	}
	return false;
}

int16_t RunTestApp::Analyze(){
	Car *car = GetSystemRes()->car;

	double x_sum = 0;
	double near_sum =0;
	double mid_sum =0;
	double far_sum =0;

	avg_width=0;

	std::vector<std::pair<int16_t, int16_t>>midpoint;
	for(int k=0; k<car->GetCameraH(); k++){
		midpoint.push_back(std::make_pair(k,(left[k].second+right[k].second)/2));

		if(right[k].second-left[k].second>0)
			avg_width += right[k].second-left[k].second;
	}
	avg_width /= car->GetCameraH();

	int16_t prev=midpoint[0].second;
	bool RightAngle=false;

	//int16_t x_count = 0;
	int16_t black_count = 0;

	for (int far=0; far<midpoint.size()/3; far++){
		if(midpoint[far].second==0){
			black_count++;
		}
		if((midpoint[far].second-prev>car->GetCameraW()/4 && midpoint[far].second-prev<car->GetCameraW()/2)&&car->GetServo().GetDegree()>=950){
			RightAngle=true;
			return 1000+far;
		}
		far_sum += midpoint[far].second;
		prev=midpoint[far].second;
	}
	x_sum += far_sum*0.1;

	for (int mid=midpoint.size()/3; mid<midpoint.size()/3*2; mid++){
		if(midpoint[mid].second==0){
			black_count++;
		}
		if((midpoint[mid].second-prev>car->GetCameraW()/4 && midpoint[mid].second-prev<car->GetCameraW()/2)&&car->GetServo().GetDegree()>=950){
			RightAngle=true;
			return 1000+mid;
		}
		mid_sum += midpoint[mid].second;
		prev=midpoint[mid].second;
	}
	if(black_count>midpoint.size()*3/5 && !RightAngle)
		x_sum += mid_sum*0.2;
	else
		x_sum += mid_sum*0.7;

	for (int near=midpoint.size()/3*2; near<midpoint.size(); near++){
		//		if(midpoint[near].second==0){
		//			black_count++;
		//		}
		if((midpoint[near].second-prev>car->GetCameraW()/4 && midpoint[near].second-prev<car->GetCameraW()/2)&&car->GetServo().GetDegree()>=950){
			RightAngle=true;
			return 1000+near;
		}
		near_sum += midpoint[near].second;
		prev=midpoint[near].second;
	}
	if(black_count>midpoint.size()*3/5 && !RightAngle)
		x_sum += near_sum*0.7;
	else
		x_sum += near_sum*0.2;

	//	char mp_buffer[100];
	//	sprintf(mp_buffer,"x_avg: %f\n", x_sum/60);
	//	car->GetUart().SendStr(mp_buffer);

	//	if((RightAngle && car->GetServo().GetDegree()>950)){
	//			return 1250;
	//	}
	//	else if(RightAngle && car->GetServo().GetDegree()<950)
	//		return 600;

	return -(x_sum/car->GetCameraH()-MIDPOINT)*FACTOR;
}

void RunTestApp::FindMargin(){

	Car *car = GetSystemRes()->car;

	std::vector<std::pair<int16_t, int16_t>> current_left;
	std::vector<std::pair<int16_t, int16_t>> current_right;

	for(int16_t image_row=0; image_row<car->GetCameraH(); image_row++){

		bool prev = true;

		for(int16_t byte=0; byte<10; byte++){

			Byte check_image = image[image_row*10+byte];

			/*to bit*/
			prev = check_image & 0x01;
			check_image >>= 1;

			for(int16_t bit=1; bit<8; bit++){

				/*if 0->1 or 1->0, check if error*/
				if (!(check_image & prev) && image_row<(car->GetCameraH()-3)){
					if(!is_error(image_row,byte*10+bit,prev)){
						if(prev) //1 -> 0
							current_left.push_back(std::make_pair(image_row,byte*10+bit));
						else //0 -> 1
							current_right.push_back(std::make_pair(image_row,byte*10+bit));
					}
					check_image >>= 1;
				}

				/*for first 3 rows, no data to check*/
				// if more than 1 left/right margin detected, must have error
				else if (!(check_image & prev)){
					if(prev && !(current_left.size()<image_row+1)){
						current_left.push_back(std::make_pair(image_row,byte*10+bit));
					}
					else if (!prev && !(current_right.size()<image_row+1)){
						current_right.push_back(std::make_pair(image_row,byte*10+bit));
					}
					else {//error detected for first 3 lines
						if(current_right[image_row].second-current_left[image_row].second < avg_width/2){
							if(prev){
								current_left.pop_back();
								current_left.push_back(std::make_pair(image_row,byte*10+bit));
							}
							else{
								current_right.pop_back();
								current_right.push_back(std::make_pair(image_row,byte*10+bit));
							}
						}
					}
					check_image >>= 1;
				}

				/*goto next*/
				else {
					check_image >>= 1;
				}

			}
		}
		if(current_left.size()<image_row+1)
			current_left.push_back(std::make_pair(image_row, 0));
		if(current_right.size()<image_row+1){
			if(prev)
				current_right.push_back(std::make_pair(image_row,0));
			else
				current_right.push_back(std::make_pair(image_row,car->GetCameraW()));
		}
	}

	left.clear();
	right.clear();

	for(int i=0; i<60; i++){

		left[i].first = current_left[i].first;
		left[i].second = current_left[i].second;

		right[i].first = current_right[i].first;
		right[i].second = current_right[i].second;

	}

}

bool RunTestApp::is_error(int16_t row, int16_t position, bool side /*left = true, right=false*/){
	// compare with previous image
	// near  far  far  near
	if(row>0){
		if (side){
			return (left[row-3].second > position) && (left[row-3].second-position < 5);
		}
		else
			return (right[row-3].second < position) && ( position - right[row-3].second < 5);
	}
	return false;

}

}

