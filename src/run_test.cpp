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
//#define MIDPOINT 38
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
	writer.WriteString("Right Angle?");

	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> x_avg =
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
		car->GetLcd().SetRegion({0, 144, St7735r::GetW(),
			LcdTypewriter::GetFontH()});
		FindMargin(image2.get());
		Analyze();
		//		if(Analyze()<1000){
		writer.WriteString(String::Format("%ld\n",
				white_after_black).c_str());/*String::Format("%ld\n",
				white_after_blackAnalyze()/FACTOR+MIDPOINT).c_str());*/
		//		}
		//		else if(Analyze()>=1000){
		//			writer.WriteString(String::Format("RA at: %ld\n",
		//					Analyze()-1000).c_str());
		//		}
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
		bool triggered = false;
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

	//		car->SetMotorPower(0,300);
	//		car->SetMotorPower(1,300);
	car->GetCamera().Start();
	looper.ResetTiming();
	while (!car->GetButton(1).IsDown())
	{
		if (car->GetCamera().IsAvailable())
		{
			memcpy(image2.get(), car->GetCamera().LockBuffer(), image_size);
			car->GetCamera().UnlockBuffer();

//			car->SetMotorPower(0,speed.speedCal(car));
//			car->SetMotorPower(1,speed.speedCal(car));


			//			car->GetLcd().SetRegion({0, 0, car->GetCameraW(), car->GetCameraH()});
			//			car->GetLcd().FillBits(0, 0xFFFF, image2.get(),
			//					car->GetCameraW() * car->GetCameraH());
			FindMargin(image2.get());

			for(Uint i=0; i<car->GetCameraH(); i++){
				car->GetLcd().SetRegion({MIDPOINT, i, 1, 1});
				car->GetLcd().FillColor(St7735r::kCyan);
			}

			/*SERVO_MID_DEGREE + (percentage_ * 370 / 1000)*/
			if(950+Analyze()*370/1000-car->GetServo().GetDegree()<200 || RightAngle || out_indicator>0)
				car->SetTurning(Analyze());

			printMidpoint();
			printMargin();
			char received;
			if(car->GetUart().PeekChar(&received)){
				if(received == 'a')
					MIDPOINT++;
				else if (received == 'b')
					MIDPOINT--;
			}
			char buffer[100];
			sprintf(buffer,"Midpoint is at %d",MIDPOINT);
			car->GetUart().SendStr(buffer);

			//			initiate = true;
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

int Mdeian(bool* array){
	int sort[5];
	for(int i = 0; i < 5; i++){
		for(int j = 0; j < 5; j++){
			if(j > i){
				sort[i] = array[j] < array[i] ? array[j] : array[i];
			}
		}
	}
	return sort[2];
}


void RunTestApp::MedianFilter(bool* array_row, int length){
	bool* pRow = array_row;
	int data[length];
	for(int i = 0; i < length; i++){
		data[i] = Mdeian(pRow++);
	}
	for(int i = 0; i < length; i++){
		array_row[i] = data[i];
	}

}

int RunTestApp::Analyze(void){
	RightAngle = false;

	Car *car = GetSystemRes()->car;
	double error = 0;

	int far_e = MIDPOINT - MidpointSumCal(0,car->GetCameraH()/3)/(car->GetCameraH()/3);
	int mid_e = MIDPOINT - MidpointSumCal(car->GetCameraH()/3,car->GetCameraH()/3*2)/(car->GetCameraH()/3);
	int near_e = MIDPOINT - MidpointSumCal(car->GetCameraH()/3*2,car->GetCameraH())/(car->GetCameraH()/3);

	double avg_error = far_e+mid_e+near_e/3;

	//if it's going out of bound
	if(check_going_out()>0){
		if(out_indicator == 1) //going out to right, turn left
			return 1000;
		else if(out_indicator ==2) //going out to left, turn right
			return -1000;
	}

	if(black_count>car->GetCameraH()*3/5 && (!RightAngle && avg_error>5)){
		error = far_e*0.1
				+ mid_e*0.8
				+ near_e*0.1;
	}
	else
		error = far_e*0.2
		+ mid_e*0.5
		+ near_e*0.3;

	if(RightAngle && car->GetServo().GetDegree()>890)
		return 1000;

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

		bool* array_ptr = bitmap[filter_row];
		MedianFilter(array_ptr,car->GetCameraW());


		car->GetLcd().SetRegion(libsc::Lcd::Rect(0,filter_row,car->GetCameraW(),1));
		car->GetLcd().FillBits(0,0xFFFF,array_ptr,car->GetCameraW());
	}

	for(int16_t row=0; row<car->GetCameraH(); row++){
		margin[row][0] = 5;
		bool l_prev = bitmap[row][(car->GetCameraH()-5)/2];

		margin[row][1] = car->GetCameraW()-5;
		bool r_prev = bitmap[row][(car->GetCameraH()-5)/2];

		for(int l_column= midpoint[row]; l_column>5 ; l_column--){
			if(bitmap[row][l_column]!=l_prev && !l_prev){
				margin[row][0]=l_column;
			}
			l_prev = bitmap[row][l_column];
		}
		for(int r_column=midpoint[row]; r_column<car->GetCameraW()-5; r_column++){
			if(bitmap[row][r_column]!=r_prev && !r_prev){
				margin[row][1]=r_column;
			}
			r_prev = bitmap[row][r_column];
		}
		if(margin[row][1]==car->GetCameraW()-5){
			if(bitmap[row][car->GetCameraW()-10])
				margin[row][1]=5;
		}

	}// for row

	//calculate midpoint
	black_count = 0;
	for(int k=0; k<car->GetCameraH(); k++){
		midpoint[k] = (margin[k][0]+margin[k][1])/2;
		if (midpoint[k]==0)
			black_count++;
	}

}// end of function

double RunTestApp::MidpointSumCal(int start, int end){
	Car *car = GetSystemRes()->car;
	//int prev = midpoint[start];
	int black_line =0;
	double sum = midpoint[start];
	white_after_black = 0;

	for (int i=start; i<end; i++){

		if(midpoint[i]==5){
			black_line=i;
		}
		sum += midpoint[i];
		//prev=midpoint[i];
	}
	//detect right angle
	if(black_line!=0){
		for(int i=0; i<3; i++){
			white_after_black += car->GetCameraW() - margin[black_line+i][1];
		}

		if(white_after_black< 200){
			RightAngle=true;
			car->GetLed(3).SetEnable(true);
		}
	}
	return sum;
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
		car->GetLcd().SetRegion({margin[i][1], i, 1, 1});
		car->GetLcd().FillColor(St7735r::kBlue);
	}
}

bool RunTestApp::check_going_out(){
	Car *car = GetSystemRes()->car;
	int left_black = 0;
	int right_black = 0;
	for(int i=0; i<car->GetCameraH(); i++){
		if(margin[i][0]==5){	//left is white,going to right
			if(margin[i][1]<MIDPOINT)	//right is black
				right_black++;
		}
		else if(margin[i][1]==car->GetCameraW()-5){	//right is white, going to left
			if(margin[i][0]>MIDPOINT)	//left is black
				left_black++;
		}
	}
	if(right_black>car->GetCameraH()/2){
		out_indicator = 1; //turn left!
	}
	else if(left_black>car->GetCameraH()/2){
		out_indicator = 2; //turn right!
	}
}

}
