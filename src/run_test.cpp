///*
// * run_test_app.cpp
// *
// * Author: Ben Lai, Ming Tsang, Peggy Lau
// * Copyright (c) 2014-2015 HKUST SmartCar Team
// * Refer to LICENSE for details
// */
//
//#include <cstdio>
//#include <cstring>
//#include <memory>
//#include <cmath>
//
//#include <libsc/lcd_typewriter.h>
//#include <libsc/k60/led.h>
//#include <libsc/k60/ov7725.h>
//#include <libsc/st7735r.h>
//#include <libsc/system.h>
//#include <libsc/timer.h>
//#include <libutil/looper.h>
//#include <libutil/misc.h>
//#include <libutil/string.h>
//#include "libutil/misc.h"
//#include "libsc/joystick.h"
//
//#include "car.h"
//#include "system_res.h"
//#include "run_test.h"
//#include "ImageProcess.h"
//
////#define FACTOR 200
////#define imageProcess.midpoint 12
////#define imageProcess.midpoint 38
////#define FACTOR 170
//
//using namespace libsc;
//using namespace libsc::k60;
//using namespace libutil;
//using namespace std;
//
//namespace camera
//{
//
//RunTestApp *m_instance;
//
//RunTestApp::RunTestApp(SystemRes *res)
//: App(res),
//  speed(0,0,0),
//  imageProcess()
//{
//	RightAngle = false;
//	//		CrossRoad = false;
//	//		initiate = false;
//	black_count = 0;
//	m_instance = this;
//	//		avg_width = 40;
//
//	//		for(int i=0; i<60; i++){
//	//			midpoint[i] = 37;
//	//		}
//
//}
//
//void RunTestApp::Run()
//{
//	printf("CarTestApp\n");
//
//	Car *car = GetSystemRes()->car;
//	car->GetLcd().Clear(libutil::GetRgb565(0x33, 0xB5, 0xE5));
//
//
//	Joystick::Config config;
//	config.id = 0;
//	for (uint8_t i = 0; i < 5; i++)
//	{
//		config.listener_triggers[i] = Joystick::Config::Trigger::kBoth;
//		config.listeners[i] = Joysticklistener;
//	}
//
//	car->SetJoystickIsr(&config);
//
//	//
//	const Uint image_size = car->GetCameraW() * car->GetCameraH() / 8;
//	unique_ptr<Byte[]> image2(new Byte[image_size]);
//	//
//	Looper looper;
//
//	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> blink =
//			[&](const Timer::TimerInt request, const Timer::TimerInt)
//			{
//		car->GetLed(0).Switch();
//		looper.RunAfter(request, blink);
//			};
//	looper.RunAfter(200, blink);
//
//	LcdTypewriter::Config writer_conf;
//	writer_conf.lcd = &car->GetLcd();
//	writer_conf.bg_color = libutil::GetRgb565(0x33, 0xB5, 0xE5);
//	LcdTypewriter writer(writer_conf);
//
////	if(abs(car->GetMotor(0).GetPower())<100){
//		car->GetLcd().SetRegion({0, 64, St7735r::GetW(), LcdTypewriter::GetFontH()});
//		writer.WriteString("Encoder:");
////	}
//
//	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> encoder =
//			[&](const Timer::TimerInt request, const Timer::TimerInt)
//			{
////		if(abs(car->GetMotor(0).GetPower())<100){
//			car->UpdateAllEncoders();
//			car->GetLcd().SetRegion({0, 80, St7735r::GetW(),
//				LcdTypewriter::GetFontH()});
//			writer.WriteString(String::Format("%ld, %ld\n",
//					car->GetEncoderCount(0), car->GetEncoderCount(1)).c_str());
//			looper.RunAfter(request, encoder);
////		}
//			};
//	looper.RunAfter(250, encoder);
//
////	if(abs(car->GetMotor(0).GetPower())<100){
//		car->GetLcd().SetRegion({0, 96, St7735r::GetW(), LcdTypewriter::GetFontH()});
//		writer.WriteString("Servo:");
////	}
//
//	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> servo =
//			[&](const Timer::TimerInt request, const Timer::TimerInt)
//			{
////		if(abs(car->GetMotor(0).GetPower())<100){
//			car->GetLcd().SetRegion({0, 112, St7735r::GetW(),
//				LcdTypewriter::GetFontH()});
//			writer.WriteString(String::Format("%ld\n",
//					car->GetServo().GetDegree()).c_str());
//			looper.RunAfter(request, servo);
////		}
//			};
//	looper.RunAfter(200, servo);
//	//
//	if(abs(car->GetMotor(0).GetPower())>100){
//		car->GetLcd().SetRegion({0, 128, St7735r::GetW(), LcdTypewriter::GetFontH()});
//		writer.WriteString("error");
//	}
//
//	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> x_avg =
//			[&](const Timer::TimerInt request, const Timer::TimerInt)
//			{
////		if(abs(car->GetMotor(0).GetPower())<100){
//			car->GetLcd().SetRegion({0, 144, St7735r::GetW(),
//				LcdTypewriter::GetFontH()});
//			//		FindMargin(imageProcess.bitmap);
//			Analyze();
//			//		if(Analyze()<1000){
//			writer.WriteString(String::Format("%ld\n",
//					Analyze()/FACTOR).c_str());/*String::Format("%ld\n",
//				white_after_blackAnalyze()/FACTOR+imageProcess.midpoint).c_str());*/
//			//		}
//			//		else if(Analyze()>=1000){
//			//			writer.WriteString(String::Format("RA at: %ld\n",
//			//					Analyze()-1000).c_str());
//			//		}
////		}
//		looper.RunAfter(request, x_avg);
//			};
//	looper.RunAfter(150, x_avg);
//
////	bool no_ec_reading = car->GetEncoderCount(0)==0 && car->GetEncoderCount(1)==0;
////	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> motor_safety =
////			[&](const Timer::TimerInt request, const Timer::TimerInt)
////			{
////		car->UpdateAllEncoders();
////		if(no_ec_reading && (car->GetMotor(0).GetPower()>0||car->GetMotor(1).GetPower()>0)){
////			car->SetMotorPower(0,0);
////			car->SetMotorPower(1,0);
////		}
////		looper.RunAfter(request, motor_safety);
////			};
////	bool triggered = false;
////	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> trigger =
////			[&](const Timer::TimerInt request, const Timer::TimerInt)
////			{
////		if (!triggered)
////		{
////			if(t){
////				//if (Trigger(car->GetEncoderCount(0),car->GetEncoderCount(1))){
////				car->SetMotorPower(0,setpower);
////				car->SetMotorPower(1,setpower);
////				triggered=true;
////				//				looper.RunAfter(1005, motor_safety);
////				looper.RunAfter(3000, trigger);
////			}
////			else
////			{
////				looper.RunAfter(500, trigger);
////			}
////		}
////		else if (triggered){
////			car->SetMotorPower(0,0);
////			car->SetMotorPower(1,0);
////			triggered=false;
////			t = false;
////			looper.RunAfter(2000, trigger);
////		}
////			};
////	looper.RunAfter(500, trigger);
//
//	car->GetCamera().Start();
//	looper.ResetTiming();
//	while (!car->GetButton(1).IsDown())
//	{
//		if (car->GetCamera().IsAvailable())
//		{
//			memcpy(image2.get(), car->GetCamera().LockBuffer(), image_size);
//			car->GetCamera().UnlockBuffer();
//
//			//						car->SetMotorPower(0,speed.speedCal(car));
//			//						car->SetMotorPower(1,speed.speedCal(car));
//			car->SetMotorPower(0,setpower);
//			car->SetMotorPower(1,setpower);
//
//			//
//			//			car->GetLcd().SetRegion({0, 0, car->GetCameraW(), car->GetCameraH()});
//			//			car->GetLcd().FillBits(0, 0xFFFF, image2.get(),
//			//					car->GetCameraW() * car->GetCameraH());
//			imageProcess.start(image2.get(), &car->GetUart());
//
//
//			//			for(Uint i=0; i<car->GetCameraH(); i++){
//			//				car->GetLcd().SetRegion({imageProcess.MIDPOINT, i, 1, 1});
//			//				car->GetLcd().FillColor(St7735r::kCyan);
//			//			}
//
//			/*SERVO_MID_DEGREE + (percentage_ * 370 / 1000)*/
//			//if(950+Analyze()*370/1000-car->GetServo().GetDegree()<200 || RightAngle || out_indicator>0)
//			car->SetTurning(Analyze());
//
//			//			car->GetLcd().SetRegion(libsc::Lcd::Rect(0, 0, car->GetCameraW(), car->GetCameraH()));
//			//			car->GetLcd().FillColor(St7735r::kWhite);
//			//			if(abs(car->GetMotor(0).GetPower())<100){
//			printMidpoint();
//			printMargin();
//			//			}
//			//printProcessedImage();
//
//			//			char received;
//			//			if(car->GetUart().PeekChar(&received)){
//			//				if(received == 'a')
//			//					imageProcess.MIDPOINT++;
//			//				else if (received == 'b')
//			//					imageProcess.MIDPOINT--;
//			//				else if(received == 'e')
//			//					t = true;
//			//				else if(received == 'f'){
//			//					car->SetMotorPower(0,setpower);
//			//					car->SetMotorPower(1,setpower);
//			//				}
//			//				else if (received == 'g'){
//			//					car->SetMotorPower(0,0);
//			//					car->SetMotorPower(1,0);
//			//				}
//			//				else if (received == 'i'){
//			//					FACTOR+=5;
//			//				}
//			//				else if (received == 'j'){
//			//					FACTOR-=5;
//			//				}
//			//				else if (received == 's'){
//			//					setpower+=10;
//			//				}
//			//				else if (received == 't'){
//			//					setpower-=10;
//			//				}
////		}
//
//		//			char buffer[100];
//		//			sprintf(buffer,"FACTOR is %d\nPOWER is at %d\n",FACTOR,setpower);
//		//			car->GetUart().SendStr(buffer);
//
//		//[initiate = true;
//	}
//
//	looper.Once();
//}
//car->GetCamera().Stop();
//}
//
//bool RunTestApp::Trigger(int32_t l_encoder_reading, int32_t r_encoder_reading){
//	if(l_encoder_reading>100||r_encoder_reading>100){
//		return true;
//	}
//	return false;
//}
//
////int Mdeian(bool* array){
////	int sort[5];
////	for(int i = 0; i < 5; i++){
////		for(int j = 0; j < 5; j++){
////			if(j > i){
////				sort[i] = array[j] < array[i] ? array[j] : array[i];
////			}
////		}
////	}
////	return sort[2];
////}
////
////
////void RunTestApp::MedianFilter(bool* array_row, int length){
////	bool* pRow = array_row;
////	int data[length];
////	for(int i = 0; i < length; i++){
////		data[i] = Mdeian(pRow++);
////	}
////	for(int i = 0; i < length; i++){
////		array_row[i] = data[i];
////	}
////
////}
//
//double MultiplyRatio(double err, int FACTOR){
//
//	if(err>15 || err<-15){
//		return err*(FACTOR*1.3);
//	}
//	else if (err>10 || err <-10){
//		return err*(FACTOR*0.7);
//	}
//	else{
//		return err*FACTOR;
//	}
//	//return err;
//}
//
//int RunTestApp::Analyze(void){
//	RightAngle = false;
//
//	Car *car = GetSystemRes()->car;
//	double error = 0;
//
//	int far_e = imageProcess.MIDPOINT - MidpointSumCal(0,car->GetCameraH()/3)/(car->GetCameraH()/3);
//	int mid_e = imageProcess.MIDPOINT - MidpointSumCal(30,40)/10;
//	int near_e = imageProcess.MIDPOINT - MidpointSumCal(car->GetCameraH()/3*2,car->GetCameraH())/(car->GetCameraH()/3);
//
//	double avg_error = mid_e;
//	//
////	//if it's going out of bound
////	if(check_going_out()>0){
////		if(out_indicator == 1) //going out to right, turn left
////			return 10000;
////		else if(out_indicator ==2) //going out to left, turn right
////			return -10000;
////	}
//
//	if(black_count>car->GetCameraH()*3/5 && (!RightAngle && (avg_error>10||avg_error<-10))){
//		error = mid_e;
//		//				far_e*0.1
//		//				+ mid_e*0.8
//		//				+ near_e*0.1;
//	}
//	else
//		error = mid_e;
//	//				far_e*0.2
//	//		+ mid_e*0.5
//	//		+ near_e*0.3;
//
//	//	if(RightAngle && car->GetServo().GetDegree()>890)
//	//		return 1000;
//
//	return MultiplyRatio(error, FACTOR);
//}
//
////void RunTestApp::FindMidpoint(){
////
////	Car *car = GetSystemRes()->car;
////
//////	car->GetLcd().SetRegion(libsc::Lcd::Rect(0,filter_row,car->GetCameraW(),1));
//////	car->GetLcd().FillBits(0,0xFFFF,array_ptr,car->GetCameraW());
////
////	//calculate imageProcess.midpoint
//
////	}
//
////}// end of function
//
//double RunTestApp::MidpointSumCal(int start, int end){
//	Car *car = GetSystemRes()->car;
//	//int prev = imageProcess.midpoint[start];
//	int black_line =0;
//	double sum = imageProcess.midpoint[start];
//	white_after_black = 0;
//
//	black_count = 0;
//	for(int k=0; k<car->GetCameraH(); k++){
//		imageProcess.midpoint[k] = (imageProcess.margin[k][0]+imageProcess.margin[k][1])/2;
//		if (imageProcess.midpoint[k]==0)
//			black_count++;
//	}
//
//
//	for (int i=start; i<end; i++){
//
//		if(imageProcess.midpoint[i]==5){
//			black_line=i;
//		}
//		sum += imageProcess.midpoint[i];
//		//prev=imageProcess.midpoint[i];
//	}
//	//detect right angle
//	if(black_line!=0){
//		for(int i=0; i<3; i++){
//			white_after_black += car->GetCameraW() - imageProcess.margin[black_line+i][1];
//		}
//
//		if(white_after_black< 200){
//			RightAngle=true;
//			car->GetLed(3).SetEnable(true);
//		}
//	}
//	return sum;
//}
//
//void RunTestApp::Joysticklistener(const uint8_t id)
//{
//	Car *car = m_instance->GetSystemRes()->car;
//
//	char buffer[100];
//	switch (id)
//	{
//	case (int)Joystick::State::kUp:
//			while(car->GetJoystick().GetState() == Joystick::State::kUp);
//	if(m_instance->LCDmode == 1){
//		m_instance->LCDmode = 2;
//		sprintf(buffer,"LCD mode switch to 2\n");
//		car->GetUart().SendStr(buffer);
//	}
//	else {
//		m_instance->LCDmode = 1;
//
//		sprintf(buffer,"LCD mode switch to 1\n");
//		car->GetUart().SendStr(buffer);
//	}
//	break;
//	//	case (int)Joystick::State::kDown:
//	//			while(car->GetJoystick().GetState() == Joystick::State::kDown);
//	//			if(m_instance->LCDmode == 1){
//	//				m_instance->LCDmode = 2;
//	//				sprintf(buffer,"LCD mode switch to 2\n");
//	//				car->GetUart().SendStr(buffer);
//	//			}
//	//			else{
//	//				m_instance->LCDmode = 1;
//	//
//	//				sprintf(buffer,"LCD mode switch to 1\n");
//	//				car->GetUart().SendStr(buffer);
//	//			}
//	//	break;
//	default:
//		break;
//
//	}
//
//}
//
//void RunTestApp::printMidpoint(){
//	Car *car = GetSystemRes()->car;
//
//	//	if(LCDmode==1){
//	bool* array_ptr;
//	for (int i=0; i<car->GetCameraH();i++){
//		array_ptr= imageProcess.bitmap[i];
//		car->GetLcd().SetRegion(libsc::Lcd::Rect(0,i,car->GetCameraW(),1));
//		car->GetLcd().FillBits(0,0xFFFF,array_ptr,car->GetCameraW());
//	}
//	//	}
//	//	else if(LCDmode==2){
//	//		car->GetLcd().SetRegion(libsc::Lcd::Rect(0,0,car->GetCameraW(),car->GetCameraH()));
//	//		car->GetLcd().Clear();
//	//		car->GetLcd().FillColor(St7735r::kWhite);
//	//	}
//
//
//	for(Uint i=0; i<car->GetCameraH(); i++){
//		car->GetLcd().SetRegion({imageProcess.MIDPOINT, i, 1, 1});
//		car->GetLcd().FillColor(St7735r::kCyan);
//	}
//
//	//	for(Uint i=0; i<car->GetCameraH(); i++){
//	//		car->GetLcd().SetRegion({imageProcess.midpoint[i], i, 1, 1});
//	//		car->GetLcd().FillColor(St7735r::kRed);
//	//	}
//	for(Uint i=30; i<40; i++){
//		car->GetLcd().SetRegion({imageProcess.midpoint[i], i, 1, 1});
//		car->GetLcd().FillColor(St7735r::kRed);
//	}
//
//
//}
//
//void RunTestApp::printMargin(){
//	Car *car = GetSystemRes()->car;
//
//	for(Uint i=0; i<car->GetCameraH(); i++){
//		car->GetLcd().SetRegion({imageProcess.margin[i][0], i, 1, 1});
//		car->GetLcd().FillColor(St7735r::kBlue);
//		car->GetLcd().SetRegion({imageProcess.margin[i][1], i, 1, 1});
//		car->GetLcd().FillColor(St7735r::kBlue);
//	}
//}
//
//void RunTestApp::printProcessedImage(){
//	Car *car = GetSystemRes()->car;
//
//	for(Uint i=0; i<car->GetCameraH(); i++){
//		bool* ptr = imageProcess.bitmap[i];
//		car->GetLcd().SetRegion(libsc::Lcd::Rect(0,i,car->GetCameraW(),1));
//		car->GetLcd().FillBits(0,0xFFFF,ptr,car->GetCameraW());
//	}
//
//}
//
//bool RunTestApp::check_going_out(){
//	Car *car = GetSystemRes()->car;
//	int left_black = 0;
//	int right_black = 0;
//
//	for(int i=30; i<40; i++){
//		if(imageProcess.margin[i][0]==5){	//left is white,going to right
//			if(imageProcess.margin[i][1]<imageProcess.MIDPOINT)	//right is black
//				right_black++;
//		}
//		else if(imageProcess.margin[i][1]==car->GetCameraW()-5){	//right is white, going to left
//			if(imageProcess.margin[i][0]>imageProcess.MIDPOINT)	//left is black
//				left_black++;
//		}
//	}
//
//
//	if(right_black>8){
//		out_indicator = 1; //turn left!
//	}
//	else if(left_black>8){
//		out_indicator = 2; //turn right!
//	}
//}
//
//}
