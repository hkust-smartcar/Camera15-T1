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


#include "libbase/k60/pit.h"

#include "libbase/k60/clock_utils.h"

using namespace libsc;
using namespace libsc::k60;
using namespace libutil;
using namespace std;

namespace camera
{

RunTestApp *m_instance;

RunTestApp::RunTestApp(SystemRes *res)
: App(res),

  s_kp(0.4f), //0.4, 0.03
  s_ki(0.0f),
  s_kd(0.045f),

  s_setpoint(0.0f),

  //19ms
  l_kp(0.0125f),
  l_ki(0.0f),
  l_kd(0.0004f),

  r_kp(0.0099f),
  r_ki(0.0f),
  r_kd(0.0004f),

  //19 ms
  l_m_setpoint(2000.0f), //2900
  r_m_setpoint(2000.0f),

  sd_setpoint(2000),


  show_error(0.0),

  servo_Control(&s_setpoint, &s_kp, &s_ki, &s_kd, -1000, 1000),
  l_speedControl(&l_m_setpoint, &l_kp, &l_ki, &l_kd, 0, 950),
  r_speedControl(&r_m_setpoint, &r_kp, &r_ki, &r_kd, 0, 950),

  m_peter(),

  m_start(0),
  m_is_stop(false)

{
	//raw data: left & right encoder, servo angle
	ec0 = 0;
	ec1 = 0;
	s_degree = 0;

	l_result = 0;
	r_result = 0;
	s_result = 0;

	sp_storage[0] = l_m_setpoint;
	sp_storage[1] = r_m_setpoint;

	m_instance = this;

//for grapher use
	m_peter.addWatchedVar(&sd_setpoint);
	m_peter.addWatchedVar(&show_error);

// servo
	m_peter.addSharedVar(&s_kp,"skp");
//	m_peter.addSharedVar(&s_ki,"ski");
	m_peter.addSharedVar(&s_kd,"skd");

	m_peter.Init(&PeggyListener);
}

//void RunTestApp::updateSPD(float error){
//
//}

void RunTestApp::DetectEmergencyStop(){

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
	if (!is_startup && (abs(count) + abs(count2) < 100))
	{
		if (m_emergency_stop_state.is_triggered)
		{
			if (Timer::TimeDiff(time, m_emergency_stop_state.trigger_time) > 150)
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
	printf("CarTestApp\n");

	//Get car
	Car *car = GetSystemRes()->car;

	//Get LCD
	car->GetLcd().Clear(libutil::GetRgb565(0x33, 0xB5, 0xE5));

	//Get image size (80*60)/byte_size
	const uint16_t image_size = car->GetCameraW() * car->GetCameraH() / 8;
	unique_ptr<Byte[]> image2(new Byte[image_size]);

//	Pit m_pit(GetPitConfig(0, std::bind(&RunTestApp::peggy, this, std::placeholders::_1)));

	Looper looper;

	//Update encoder count, input to and update speed PID controller
		std::function<void(const Timer::TimerInt, const Timer::TimerInt)> encoder =
				[&](const Timer::TimerInt, const Timer::TimerInt)
				{
					//update and get encoder's count
					car->UpdateAllEncoders();
					ec0 =  car->GetEncoderCount(0);
					ec1 =  car->GetEncoderCount(1);

					speed = ec0/57300.0f/0.019f;

					//if t is true, update PID and give power to car, else stop the car
					if(t && !m_is_stop){
						l_result = (int32_t)l_speedControl.updatePID((float)car->GetEncoderCount(0));
						car->SetMotorPower(0,l_result);

						r_result = (int32_t)r_speedControl.updatePID((float)car->GetEncoderCount(1));
						car->SetMotorPower(1,r_result);

//						DetectEmergencyStop();
					}
					else {
						car->SetMotorPower(0,0);
						car->SetMotorPower(1,0);
					}
					//
				};
		looper.Repeat(19, encoder, Looper::RepeatMode::kLoose);
	//	looper.Repeat(89, encoder, Looper::RepeatMode::kPrecise);


	//breathing led- indicate if program hang or not
	looper.Repeat(199, std::bind(&libsc::Led::Switch, &car->GetLed(0)), Looper::RepeatMode::kLoose);

	//Send data to grapher
	looper.Repeat(31, std::bind(&MyVarManager::sendWatchData, &m_peter), Looper::RepeatMode::kLoose);


	//Update servo error, input to and update servo PID controller
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
		imageProcess.blp.Analyze(imageProcess.bitmap);
//		printResult();

		//set angle with servo PID controller and image process result
		//negative for correcting direction
		//
		float error = (float)imageProcess.Analyze();
//		if(imageProcess.STATE != 5){
//			updateSPD(error);
//		}
//		else{
//			s_kp = SKP;
//			s_kd = SKD;
//
//		}

		show_error = error;//abs((int)(error/1000));
		s_result = (int16_t)servo_Control.updatePID_ori(-error);
		car->SetTurning(s_result);
		if(abs(s_result)<10000){
		l_m_setpoint =  (float)software_differential.turn_left_encoder(sd_setpoint,car->GetServo().GetDegree(),9500,3700);
		r_m_setpoint = (float) software_differential.turn_right_encoder(sd_setpoint,car->GetServo().GetDegree(),9500,3700);
		}
		else{
			l_m_setpoint =  sd_setpoint;
			r_m_setpoint = sd_setpoint;
		}
	};
	looper.Repeat(11, servo, Looper::RepeatMode::kPrecise);


	//Get image
	car->GetCamera().Start();
	looper.ResetTiming();
	while (!car->GetButton(1).IsDown())
	{
		looper.Once();
	}
	car->GetCamera().Stop();

}



void RunTestApp::printResult(){
	Car *car = GetSystemRes()->car;

	//Initiate LCD writer for printing real time information
	LcdTypewriter::Config writer_conf;
	writer_conf.lcd = &car->GetLcd();
	writer_conf.bg_color = libutil::GetRgb565(0x33, 0xB5, 0xE5);
	LcdTypewriter writer(writer_conf);

	//print filtered image
	for(uint16_t i=0; i<car->GetCameraH(); i++)
	{
		bool* ptr = imageProcess.bitmap[i];
		car->GetLcd().SetRegion(libsc::Lcd::Rect(0,i,car->GetCameraW(),1));
		car->GetLcd().FillBits(0,0xFFFF,ptr,car->GetCameraW());
	}

	//
	// print margin found
	for(uint16_t i=0; i<car->GetCameraH(); i++)
	{
		car->GetLcd().SetRegion({imageProcess.margin[i][0], i, 1, 1});
		car->GetLcd().FillColor(St7735r::kBlue);
		car->GetLcd().SetRegion({imageProcess.margin[i][1], i, 1, 1});
		car->GetLcd().FillColor(St7735r::kBlue);
	}
	//
	//

	// print margin found
	for(uint16_t i=0; i<car->GetCameraH(); i++)
	{
		car->GetLcd().SetRegion({imageProcess.blp.margin[i][0], i, 1, 1});
		car->GetLcd().FillColor(St7735r::kYellow);
		car->GetLcd().SetRegion({imageProcess.blp.margin[i][1], i, 1, 1});
		car->GetLcd().FillColor(St7735r::kYellow);
	}

//	car->GetLcd().SetRegion({0, 64, St7735r::GetW(), LcdTypewriter::GetFontH()});
//	writer.WriteString(String::Format("%ld, %ld\n",imageProcess.blp.margin[10][0], imageProcess.blp.margin[10][1]).c_str());
//
//	car->GetLcd().SetRegion({0, 80, St7735r::GetW(), LcdTypewriter::GetFontH()});
//	writer.WriteString(String::Format("%ld, %ld\n",imageProcess.blp.margin[20][0], imageProcess.blp.margin[20][1]).c_str());
//
//	car->GetLcd().SetRegion({0, 96, St7735r::GetW(), LcdTypewriter::GetFontH()});
//	writer.WriteString(String::Format("%ld, %ld\n",imageProcess.blp.margin[30][0], imageProcess.blp.margin[30][1]).c_str());
//
//	car->GetLcd().SetRegion({0, 112, St7735r::GetW(), LcdTypewriter::GetFontH()});
//	writer.WriteString(String::Format("%ld, %ld\n",imageProcess.blp.margin[40][0], imageProcess.blp.margin[40][1]).c_str());
//	//

	//

 	 //print midpoint
	for(uint16_t i=0; i<car->GetCameraH(); i++){
			car->GetLcd().SetRegion({MIDPOINT_REF, i, 1, 1});
			car->GetLcd().FillColor(St7735r::kCyan);
	}

	for(uint16_t i=0; i<car->GetCameraH(); i++){

		car->GetLcd().SetRegion({imageProcess.midpoint[i], i, 1, 1});
		car->GetLcd().FillColor(St7735r::kRed);
	}
//	if (imageProcess.crossroad)
//		{
//
//		for(uint16_t i=libutil::Clamp(0,(int)(imageProcess.white_start-1),60); i<libutil::Clamp(0,(int)imageProcess.white_start-11,60); i++){
//			car->GetLcd().SetRegion({imageProcess.midpoint[i], i, 1, 1});
//			car->GetLcd().FillColor(St7735r::kRed);
//		}
//
//		}
//	else if(imageProcess.black_line){
//		if(imageProcess.white_end-imageProcess.white_start>10){
//			for(uint16_t i=libutil::Clamp(0,(int)(imageProcess.black_line_end+1),60); i<libutil::Clamp(0,(int)imageProcess.black_line_end+11,60); i++){
//				car->GetLcd().SetRegion({imageProcess.midpoint[i], i, 1, 1});
//				car->GetLcd().FillColor(St7735r::kRed);
//			}
//		}
//		else{
//			for(uint16_t i=35; i<45; i++){
//				car->GetLcd().SetRegion({imageProcess.midpoint[i], i, 1, 1});
//				car->GetLcd().FillColor(St7735r::kRed);
//			}
//		}
//
//	}
//	else
//	{
//		for(uint16_t i=35; i<45; i++){
//			car->GetLcd().SetRegion({imageProcess.midpoint[i], i, 1, 1});
//			car->GetLcd().FillColor(St7735r::kRed);
//		}
//	}

	//print Q and cross road related info

	car->GetLcd().SetRegion(libsc::Lcd::Rect(0,imageProcess.black_end,car->GetCameraW(),1));
	car->GetLcd().FillColor(St7735r::kGreen);

	car->GetLcd().SetRegion(libsc::Lcd::Rect(0,imageProcess.black_line_start,car->GetCameraW(),1));
	car->GetLcd().FillColor(St7735r::kGreen);

	car->GetLcd().SetRegion(libsc::Lcd::Rect(0,imageProcess.black_line_end,car->GetCameraW(),1));
	car->GetLcd().FillColor(St7735r::kGreen);

	car->GetLcd().SetRegion(libsc::Lcd::Rect(0,imageProcess.white_start,car->GetCameraW(),1));
	car->GetLcd().FillColor(St7735r::kGreen);

	car->GetLcd().SetRegion(libsc::Lcd::Rect(0,imageProcess.white_end,car->GetCameraW(),1));
	car->GetLcd().FillColor(St7735r::kGreen);

	//BE,WS, WE for Q & crossroad
	car->GetLcd().SetRegion({0, 64, St7735r::GetW(), LcdTypewriter::GetFontH()});
	writer.WriteString(String::Format("%ld, %ld, %ld,%ld\n",imageProcess.black_end, imageProcess.checkRA, imageProcess.white_start, imageProcess.white_end).c_str());

	car->GetLcd().SetRegion({0, 80, St7735r::GetW(), LcdTypewriter::GetFontH()});
	if(imageProcess.black_line){
		writer.WriteString(String::Format("BLBLBL: %ld, %ld",imageProcess.black_line_start, imageProcess.black_line_end).c_str());
	}
	else
		writer.WriteString(String::Format("!BL!BL!BL: %ld, %ld",imageProcess.black_line_start, imageProcess.black_line_end).c_str());

	car->GetLcd().SetRegion({0, 96, St7735r::GetW(), LcdTypewriter::GetFontH()});
	if(imageProcess.bg){
		writer.WriteString(String::Format("BGBGBG: %ld",imageProcess.blp.narrow_count).c_str());
	}
	else
		writer.WriteString(String::Format("!BG!BG!BG: %ld",imageProcess.blp.narrow_count).c_str());

	car->GetLcd().SetRegion({0, 112, St7735r::GetW(), LcdTypewriter::GetFontH()});
	if(imageProcess.crossroad){
		writer.WriteString("XXX");
	}
	else{
		writer.WriteString("!X!X!X");
	}

	car->GetLcd().SetRegion({0, 128, St7735r::GetW(), LcdTypewriter::GetFontH()});
	if(imageProcess.right_angle){
		writer.WriteString("RRR");
	}
	else{
		writer.WriteString("!R!R!R");
	}

	car->GetLcd().SetRegion(libsc::Lcd::Rect(0,144, St7735r::GetW(),LcdTypewriter::GetFontH()));
//	writer.WriteString(String::Format("%ld, %ld\n",imageProcess.black_line_start, imageProcess.black_line_end).c_str());
//	writer.WriteString(String::Format("%f",imageProcess.slope).c_str());
	writer.WriteString(String::Format("%ld",imageProcess.Analyze()).c_str());

	//
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
		else{

			m_instance->m_start = System::Time();
			m_instance->m_is_stop = false;
			m_instance->t = true;
		}
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
		m_instance->sp_storage[0] = m_instance->l_m_setpoint;
		m_instance->sp_storage[1] = m_instance->r_m_setpoint;
		m_instance->l_m_setpoint = -3000.0f;
		m_instance->r_m_setpoint = -3000.0f;

		m_instance->t = true;
		break;

	//restore to original sp, stop moving backward
	case 'Y':
		m_instance->l_m_setpoint = m_instance->sp_storage[0];
		m_instance->r_m_setpoint = m_instance->sp_storage[1];

		m_instance->t = false;
		break;
	}
}


}
