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

#include "car.h"
#include "system_res.h"
#include "run_test.h"

//#define FACTOR 200
//#define MIDPOINT 12
#define MIDPOINT 38
//#define FACTOR 170

using namespace libsc;
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
	//
	const Uint image_size = car->GetCameraW() * car->GetCameraH() / 8;
	unique_ptr<Byte[]> image2(new Byte[image_size]);
	//
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
	//
	car->GetLcd().SetRegion({0, 128, St7735r::GetW(), LcdTypewriter::GetFontH()});
	writer.WriteString("result:");

	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> x_avg =
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
		car->GetLcd().SetRegion({0, 144, St7735r::GetW(),
			LcdTypewriter::GetFontH()});
		FindMargin(image2.get());
		//		if(Analyze()<1000){
		writer.WriteString(String::Format("%ld\n",
				Analyze()/*/FACTOR+MIDPOINT*/).c_str());
		//		}
		//		else if(Analyze()>=1000){
		//			writer.WriteString(String::Format("RA at: %ld\n",
		//					Analyze()-1000).c_str());
		//		}
		looper.RunAfter(request, x_avg);
			};
	looper.RunAfter(150, x_avg);
	//
	//	bool no_ec_reading = car->GetEncoderCount(0)==0 && car->GetEncoderCount(1)==0;
	//	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> motor_safety =
	//			[&](const Timer::TimerInt request, const Timer::TimerInt)
	//			{
	//		car->UpdateAllEncoders();
	//		if(no_ec_reading && (car->GetMotor(0).GetPower()>0||car->GetMotor(1).GetPower()>0)){
	//			car->SetMotorPower(0,0);
	//			car->SetMotorPower(1,0);
	//		}
	//		looper.RunAfter(request, motor_safety);
	//			};
	//	bool triggered = false;
	//	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> trigger =
	//			[&](const Timer::TimerInt request, const Timer::TimerInt)
	//			{
	//		if (!triggered)
	//		{
	//			if (Trigger(car->GetEncoderCount(0),car->GetEncoderCount(1))){
	//				car->SetMotorPower(0,250);
	//				car->SetMotorPower(1,250);
	//				triggered=true;
	//				//				looper.RunAfter(1005, motor_safety);
	//				looper.RunAfter(3000, trigger);
	//			}
	//			else
	//			{
	//				looper.RunAfter(500, trigger);
	//			}
	//		}
	//		else if (triggered){
	//			car->SetMotorPower(0,0);
	//			car->SetMotorPower(1,0);
	//			triggered=false;
	//			looper.RunAfter(2000, trigger);
	//		}
	//			};
	//	looper.RunAfter(500, trigger);

	//	car->SetMotorPower(0,200);
	//	car->SetMotorPower(1,200);
	car->GetCamera().Start();
	looper.ResetTiming();
	while (!car->GetButton(1).IsDown())
	{
		if (car->GetCamera().IsAvailable())
		{
			memcpy(image2.get(), car->GetCamera().LockBuffer(), image_size);
			car->GetCamera().UnlockBuffer();

//			car->GetLcd().SetRegion({0, 0, car->GetCameraW(), car->GetCameraH()});
//			car->GetLcd().FillBits(0, 0xFFFF, image2.get(),
//					car->GetCameraW() * car->GetCameraH());
			FindMargin(image2.get());

			for(Uint i=0; i<car->GetCameraH(); i++){
				car->GetLcd().SetRegion({MIDPOINT, i, 1, 1});
				car->GetLcd().FillColor(St7735r::kCyan);
			}


			/*SERVO_MID_DEGREE + (percentage_ * 370 / 1000)*/
			car->SetTurning(Analyze());
			//printMidpoint();
			//printMargin();
			//char received;
			//			bt.PeekChar(received){
			//
			//			}


			initiate = true;
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

int* RunTestApp::MedianFilter(bool array_row[]){
	Car *car = GetSystemRes()->car;
	int* result = new int[car->GetCameraW()];
	bool* p_array = array_row;
	//int frame[5];
	int count = 0;
	result[0] = array_row[0];
	result[1] = array_row[1];

	while(count*5+2 < car->GetCameraW()){

		//		for(int i=0; i<5; i++){
		//			frame[0] = *p_array;
		//			frame[1] = *(p_array + 1);
		//			frame[2] = *(p_array + 2);
		//			frame[3] = *(p_array + 3);
		//			frame[4] = *(p_array + 4);
		//		}
		//
		//		result[count*5+2]=frame[2]; //median
		int sum = *p_array+*(p_array + 1)+*(p_array + 2)+*(p_array + 3)+*(p_array + 4);
		if(sum>=3)
			result[count*5+2] = 1;
		else
			result[count*5+2] = 0;
		p_array = p_array+1;

	}

	return result;
}

int RunTestApp::Analyze(void){
	Car *car = GetSystemRes()->car;
	double error = 0;
	cal_midpoint();

	int far_e = MIDPOINT - AvgCal(0,car->GetCameraH()/3)/(car->GetCameraH()/3);
	int mid_e = MIDPOINT - AvgCal(car->GetCameraH()/3,car->GetCameraH()/3*2)/(car->GetCameraH()/3);
	int near_e = MIDPOINT - AvgCal(car->GetCameraH()/3*2,car->GetCameraH())/(car->GetCameraH()/3);

	double avg_error = far_e+mid_e+near_e/3;

	//if it's going out of bound
	if(black_count>car->GetCameraH()*4/5 && midpoint[0]==0){
		return 100;
	}

	if(black_count>car->GetCameraH()*4/5 && midpoint[car->GetCameraW()-1]==0){
		return -100;
	}

	if(black_count>car->GetCameraH()*3/5 && (!RightAngle && avg_error>3)){
		error = far_e*0.1
				+ mid_e*0.3
				+ near_e*0.7;
	}
	else
		error = far_e*0.2
		+ mid_e*0.5
		+ near_e*0.3;

	//int avg = AvgCal(0,car->GetCameraH())/car->GetCameraH();

	//int avg = x_sum/car->GetCameraH();
	//	if (avg>MIDPOINT){
	//	return ((MIDPOINT-avg)*FACTOR);
	//	}
	//	else{
	//		return 900+((MIDPOINT-avg)*FACTOR);
	//	}
	return error*FACTOR;
}

void RunTestApp::FindMargin(Byte* image){

	Car *car = GetSystemRes()->car;

	bool bitmap[car->GetCameraH()][car->GetCameraW()];

	for(int16_t image_row=0; image_row<car->GetCameraH(); image_row++){
		for(int16_t byte=0; byte<10; byte++){
			Byte check_image = image[image_row*10+byte];
			/*to bit*/
			for(int16_t bit=7; bit>=0; bit--){
				bitmap[image_row][8*byte+bit] = check_image & 0x01;
				check_image >>= 1;
			}
		}
	}

	for(Uint filter_row=0; filter_row<car->GetCameraH(); filter_row++){
		bool* pointer = bitmap[filter_row];
		memcpy(pointer, MedianFilter(bitmap[filter_row]), car->GetCameraW());
		car->GetLcd().SetRegion(libsc::Lcd::Rect(0,filter_row,car->GetCameraW(),1));
		car->GetLcd().FillBits(0,0xff,pointer,car->GetCameraW());
	}

	int16_t row=0;
	for(int16_t column=0; column<car->GetCameraH(); column++){
		margin[column][0] = 0;
		margin[column][1] = car->GetCameraW()-1;
		bool prev = bitmap[column][row];
		for(row=1; row<80; row++){
			if(bitmap[column][row]!=prev){
				if (prev){
					//if(!is_error(margin[image_row][0],row))
					margin[column][0]=row;
				}
				else{
					//if(!is_error(row,margin[image_row][1]))
					margin[column][1]=row;
				}
			}
			prev = bitmap[column][row];
		}
		//no color change in row, check if black row
		if(margin[column][1]==car->GetCameraW()-1){
			if (prev && margin[column][0]==0)
				margin[column][1]=0;
		}
	}// for column

}// end of function
//		for(int16_t byte=0; byte<10; byte++){
//			//check byte by byte
//			Byte check_image = image[image_row*10+byte];
//			bool check_bit = check_image & 0x01;
//
//			for(int16_t bit=7; bit>=0; bit--){
//				/*if 0<-1 or 1<-0, check if error*/
//					if (!(check_bit==prev)){
//
//						if(!prev)/*1 <- 0*/{
//							if(!is_error(margin[image_row][0],8*byte+bit))
//								margin[image_row][0] = 8*byte+bit;
//						}
//						else /*0 <- 1*/{
//							if(!is_error(8*byte+bit,margin[image_row][1]))
//								margin[image_row][1] = 8*byte+bit;
//						}
//
//					}
//
//				prev = check_bit;
//				check_image >>= 1;
//				check_bit = check_image&0x01;
//
//			} // for bit
//
//		}// for byte

bool RunTestApp::is_error(int left, int right){
	// compare with previous image
	// near  far  far  near
	if(right>left)
		return true;
	if(right-left<avg_width/2 || right-left>=79)
		return true;
	return false;

}

double RunTestApp::AvgCal(int start, int end){
	Car *car = GetSystemRes()->car;
	int prev = midpoint[start];
	double sum = midpoint[start];
	RightAngle = false;

	for (int i=start+1; i<end; i++){

		//		if(midpoint[i]-prev>car->GetCameraW()/4 && midpoint[i]-prev<car->GetCameraW()/2){
		//		//if(midpoint[i]!=0 && (margin[i+15][1] - margin[i][1]>car->GetCameraW()/2)){
		//			RightAngle=true;
		//			return 1000+i;
		//		}
		sum += midpoint[i];
		prev=midpoint[i];
	}
	return sum;
}

void RunTestApp::cal_midpoint(void){
	avg_width=0;
	int avg_count = 0;
	black_count = 0;

	Car *car = GetSystemRes()->car;

	for(int k=0; k<car->GetCameraH(); k++){
		midpoint[k] = (margin[k][0]+margin[k][1])/2;
		if (midpoint[k]==0)
			black_count++;
		if(margin[k][1]-margin[k][0]>0)
			avg_width += margin[k][1]-margin[k][0];
		avg_count++;
	}
	avg_width /= avg_count;
}

void RunTestApp::printMidpoint(){
	Car *car = GetSystemRes()->car;

	for(Uint i=0; i<car->GetCameraH(); i++){
		car->GetLcd().SetRegion({midpoint[i], i, 1, 1});
		car->GetLcd().FillColor(St7735r::kRed);
	}

}

void RunTestApp::printMargin(){
	Car *car = GetSystemRes()->car;

	for(Uint i=0; i<car->GetCameraH(); i++){
		car->GetLcd().SetRegion({margin[i][0], i, 1, 1});
		car->GetLcd().FillColor(St7735r::kBlue);
	}
	for(Uint i=0; i<car->GetCameraH(); i++){
		car->GetLcd().SetRegion({margin[i][1], i, 1, 1});
		car->GetLcd().FillColor(St7735r::kBlue);
	}
}

}
