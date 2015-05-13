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
/*89 ms
  l_kp(0.0125f),
//  l_kp(0.1f),
  l_ki(0.00000000000001f),
  l_kd(0.0f),

  r_kp(0.0099f),
  r_ki(0.00000000000001421f),
  r_kd(0.0f),
*/
  //19ms
  l_kp(0.0125f),
//  l_ki(0.0002f),
  l_ki(0.0f),
  l_kd(0.0004f),

  r_kp(0.0099f),
//  r_ki(0.0001f),
  r_ki(0.0f),
  r_kd(0.0004f),

  s_kp(0.52f), //0.43
  s_ki(0.0f),
  s_kd(0.00039f),


  /* 89 ms
  l_m_setpoint(4700.0f),
  r_m_setpoint(4700.0f),
  */

  //19 ms
  l_m_setpoint(1600.0f),
  r_m_setpoint(1600.0f),

  sd_setpoint(1600),

  s_setpoint(0.0f),

  l_speedControl(&l_m_setpoint, &l_kp, &l_ki, &l_kd, 0, 950),
  r_speedControl(&r_m_setpoint, &r_kp, &r_ki, &r_kd, 0, 950),
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

	sp_storage[0] = l_m_setpoint;
	sp_storage[1] = r_m_setpoint;

	m_instance = this;

	//for grapher use

	//watch encoder
	m_peter.addWatchedVar(&ec0);
	m_peter.addWatchedVar(&ec1);
	m_peter.addWatchedVar(&sd_setpoint);

	// servo

	m_peter.addSharedVar(&s_kp,"skp");
	m_peter.addSharedVar(&s_ki,"ski");
	m_peter.addSharedVar(&s_kd,"skd");
//	m_peter.addWatchedVar(&s_degree);
//	m_peter.addWatchedVar(&s_result);
//	m_peter.addWatchedVar(&sd_setpoint);

	//

	/* right wheel

	m_peter.addWatchedVar(&ec1);
	m_peter.addWatchedVar(&r_m_setpoint);
	m_peter.addWatchedVar(&r_kp);
	m_peter.addWatchedVar(&r_ki);
	m_peter.addWatchedVar(&r_kd);
	m_peter.addSharedVar(&r_m_setpoint,"rsp");
	m_peter.addSharedVar(&r_kp,"rkp");
	m_peter.addSharedVar(&r_ki,"rki");
	m_peter.addSharedVar(&r_kd,"rkd");

	*/

	/* left wheel

	m_peter.addWatchedVar(&ec0);
	m_peter.addWatchedVar(&l_m_setpoint);
	m_peter.addWatchedVar(&l_kp);
	m_peter.addWatchedVar(&l_ki);
	m_peter.addWatchedVar(&l_kd);
	m_peter.addSharedVar(&l_m_setpoint,"lsp");
	m_peter.addSharedVar(&l_kp,"lkp");
	m_peter.addSharedVar(&l_ki,"lki");
	m_peter.addSharedVar(&l_kd,"lkd");

	*/

	m_peter.addSharedVar(&r_kp,"rkp");
	m_peter.addSharedVar(&r_kd,"rkd");
	m_peter.addSharedVar(&l_kp,"lkp");
	m_peter.addSharedVar(&l_kd,"lkd");

	m_peter.Init(&PeggyListener);
}

void RunTestApp::Run()
{
	printf("CarTestApp\n");

	/*for checking time frame
	Gpo::Config gpo_config;
	gpo_config.pin = Pin::Name::kPte25;
	gpo_config.is_high = true;
	Gpo gpo(gpo_config);
	*/

	//Get car
	Car *car = GetSystemRes()->car;

	//Get LCD
	car->GetLcd().Clear(libutil::GetRgb565(0x33, 0xB5, 0xE5));

	//Get image size (80*60)/byte_size
	const Uint image_size = car->GetCameraW() * car->GetCameraH() / 8;
	unique_ptr<Byte[]> image2(new Byte[image_size]);

	Looper looper;

	//Update encoder count, input to and update speed PID controller
		std::function<void(const Timer::TimerInt, const Timer::TimerInt)> encoder =
				[&](const Timer::TimerInt request, const Timer::TimerInt)
				{
					//update and get encoder's count
					car->UpdateAllEncoders();
					ec0 =  car->GetEncoderCount(0);
					ec1 =  car->GetEncoderCount(1);

					//print encoder count
	//				car->GetLcd().SetRegion({0, 80, St7735r::GetW(),
	//					LcdTypewriter::GetFontH()});
	//				writer.WriteString(String::Format("%ld, %ld\n",
	//						ec0, ec1).c_str());

					/*

					car->SetMotorPower(0,180);
					car->SetMotorPower(1,180);

					*/

					//
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
					//
				};
		looper.Repeat(19, encoder, Looper::RepeatMode::kLoose);
	//	looper.Repeat(89, encoder, Looper::RepeatMode::kPrecise);


	//breathing led- indicate if program hang or not
	looper.Repeat(199, std::bind(&libsc::Led::Switch, &car->GetLed(0)), Looper::RepeatMode::kLoose);

	//Initiate LCD writer for printing real time information
		LcdTypewriter::Config writer_conf;
		writer_conf.lcd = &car->GetLcd();
		writer_conf.bg_color = libutil::GetRgb565(0x33, 0xB5, 0xE5);
		LcdTypewriter writer(writer_conf);

	/*
	 	//Print Encoder count
		car->GetLcd().SetRegion({0, 64, St7735r::GetW(), LcdTypewriter::GetFontH()});
		writer.WriteString("Encoder:");
	*/

/*
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

*/

	//Send data to grapher
	looper.Repeat(31, std::bind(&MyVarManager::sendWatchData, &m_peter), Looper::RepeatMode::kLoose);


	//Update servo error, input to and update servo PID controller
	std::function<void(const Timer::TimerInt, const Timer::TimerInt)> servo =
			[&](const Timer::TimerInt, const Timer::TimerInt)
	{
//		gpo.Set();

		if (car->GetCamera().IsAvailable())
		{
			memcpy(image2.get(), car->GetCamera().LockBuffer(), image_size);
			car->GetCamera().UnlockBuffer();
		}

		//for checking time frame
//		gpo.Turn();

		//start image processing
		imageProcess.start(image2.get());
		//printResult();

		//set angle with servo PID controller and image process result
		//negative for correcting direction
		//

		s_result = (int16_t)servo_Control.updatePID_ori(-(float)imageProcess.Analyze());
		car->SetTurning(s_result);

		//set speed according to software differential
		l_m_setpoint =  (float)software_differential.turn_left_encoder(sd_setpoint,car->GetServo().GetDegree(),9500,3700);
		r_m_setpoint = (float) software_differential.turn_right_encoder(sd_setpoint,car->GetServo().GetDegree(),9500,3700);

		//

		if (imageProcess.crossroad && abs(car->GetServo().GetDegree()-9500)<10){
			car->SetTurning(0);
		}

		/*BE,WS, WE for Q & crossroad
		car->GetLcd().SetRegion({0, 64, St7735r::GetW(), LcdTypewriter::GetFontH()});
		writer.WriteString(String::Format("%ld, %ld, %ld\n",imageProcess.black_end, imageProcess.white_start, imageProcess.white_end).c_str());

		car->GetLcd().SetRegion({0, 80, St7735r::GetW(), LcdTypewriter::GetFontH()});
		if(imageProcess.Q){
			writer.WriteString("QQQ");
		}
		else{
			writer.WriteString("!Q!Q!Q");
		}

		car->GetLcd().SetRegion({0, 96, St7735r::GetW(), LcdTypewriter::GetFontH()});
		if(imageProcess.Qr){
			writer.WriteString("QrQrQr");
		}
		else
			writer.WriteString("!Qr!Qr!Qr");

		car->GetLcd().SetRegion({0, 112, St7735r::GetW(), LcdTypewriter::GetFontH()});
		if(imageProcess.crossroad){
			writer.WriteString("XXX");
		}
		else{
			writer.WriteString("!X!X!X");
		}

		car->GetLcd().SetRegion({0, 128, St7735r::GetW(), LcdTypewriter::GetFontH()});
		if(imageProcess.l_byebye || imageProcess.r_byebye){
			writer.WriteString("bye,bye");
		}
		else{
			writer.WriteString("here,here");
		}
		*/
//		gpo.Reset();
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

	/*print margin found
	for(Uint i=0; i<car->GetCameraH(); i++)
	{
		car->GetLcd().SetRegion({imageProcess.margin[i][0], i, 1, 1});
		car->GetLcd().FillColor(St7735r::kBlue);
		car->GetLcd().SetRegion({imageProcess.margin[i][1], i, 1, 1});
		car->GetLcd().FillColor(St7735r::kBlue);
	}
	*/

	//print filtered image
	for(Uint i=0; i<car->GetCameraH(); i++)
	{
		bool* ptr = imageProcess.bitmap[i];
		car->GetLcd().SetRegion(libsc::Lcd::Rect(0,i,car->GetCameraW(),1));
		car->GetLcd().FillBits(0,0xFFFF,ptr,car->GetCameraW());
	}

	//print midpoint
	for(Uint i=30; i<40; i++){
		car->GetLcd().SetRegion({imageProcess.midpoint[i], i, 1, 1});
		car->GetLcd().FillColor(St7735r::kWhite);
	}

	//print Q and cross road related info

	car->GetLcd().SetRegion(libsc::Lcd::Rect(0,imageProcess.black_end,car->GetCameraW(),1));
	car->GetLcd().FillColor(St7735r::kGreen);

	car->GetLcd().SetRegion(libsc::Lcd::Rect(0,imageProcess.white_start,car->GetCameraW(),1));
	car->GetLcd().FillColor(St7735r::kGreen);

	car->GetLcd().SetRegion(libsc::Lcd::Rect(0,imageProcess.white_end,car->GetCameraW(),1));
	car->GetLcd().FillColor(St7735r::kGreen);

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
