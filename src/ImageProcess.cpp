/*
 * ImageProcess.cpp
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include "ImageProcess.h"
#include <algorithm>
#include <libsc/lcd_typewriter.h>
#include <libutil/string.h>

namespace camera{

ImageProcess::ImageProcess(Car* car_ptr)
:
	black_count(0),
	white_count(0),

	FACTOR(100),

	black_end(CS),
	checkRA(CS),
	white_start(CE-1),
	white_end(CE-1),
	black_line_start(CS),
	black_line_end(CS),
	double_check_black_end(CS),

	RA_dir(false),

	crossroad(false),
	l_byebye(false),
	r_byebye(false),
	right_angle(false),
	black_line(false),
	bg(false),
//	continue_right_angle(false),
	bias_crossroad(false),
	continue_bias_crossroad(false),
	first_black_line(false),

	STATE(0),
	prev_error(0)
{
	for(int i=CS; i<CE; i++){
		midpoint[i] = MIDPOINT_REF;

		data[i][0]=0;
		data[i][1]=0;
		data[i][2]=0;
		data[i][3]=LEFT;

	}

	car = car_ptr;
}

void ImageProcess::start(Byte* image){

/************************INITIALIZATION***********************/

	//for cross road
	white_start = CS;
	white_end = CE;
	black_end = CS;
	white_count = 0;
	crossroad = false;
	bias_crossroad = false;

	//for going out
	l_byebye = false;
	r_byebye = false;
	int l_going_out = 0;
	int r_going_out = 0;

	//for right angle
	right_angle = false;
	RA_dir = false;
//	continue_right_angle=false;

	//for special settings
//	if(!first_black_line){
		black_line = false;
//	}
	bg = false;

	//filter and convert to bits
	medianFilter.medianFilter(image,bitmap);

	//analyze for black guide line at the same time
	blp.Analyze(bitmap);

	//analyze vertically
	vip.Analyze(bitmap);

/************************START IMAGE PROCESSING***********************/

	for(int16_t row=CE-1; row>=CS; row--){

		//clear old data
		data[row][WHITECOUNT]= 0;
		data[row][ISWHITE] = 0;
		data[row][ISBLACK] = 0;
		data[row][WHITEAT] = LEFT;

		//collect row data: white pixels in row
		for(int pixels=RS; pixels<RE; pixels++){
			if(!bitmap[row][pixels])
				data[row][WHITECOUNT]++;
		}

		//collect row data: white row?
		if(data[row][WHITECOUNT]>RE-5){//WIDTH*5/6){
			data[row][ISWHITE] = 1;
		}

		//collect row data: black row?
		else if(WIDTH-data[row][WHITECOUNT]>RE-5){//})>WIDTH*5/6){
			data[row][ISBLACK] = 1;
		}

		//collect row data: white at left(0) or right(1)
		int lwc = 0;
		int rwc = 0;
		for(int i = RS; i<RE/2; i++){
			if(!bitmap[row][i]) //if white
				lwc++;
		}
		for(int i = RE/2; i<RE; i++){
			if(!bitmap[row][i]) //if white
				rwc++;
		}
		if(rwc>lwc){
			data[row][WHITEAT]=RIGHT;
		}
		else{
			data[row][WHITEAT]=LEFT;
		}

		int16_t mid;

		if(row == CE-1){
			mid = midpoint[row];
		}
		//prevent sudden change of midpoint
		else{
			mid = midpoint[row+1];
		}

		margin[row][LEFT]=RS;
		margin[row][RIGHT]=RE;
		//if midpoint = black, out of track already
		if(!bitmap[row][mid]){
			//start from previous midpoint
			bool l_prev = bitmap[row][mid];
			bool r_prev = bitmap[row][mid];

			// scan to left
			for(int l_column= mid; l_column>=RS ; l_column--){
				if(bitmap[row][l_column]!=l_prev && !l_prev){
					margin[row][LEFT]=l_column;
					break;
				}
				l_prev = bitmap[row][l_column];
			}

			//scan to right
			for(int r_column= mid; r_column<RE; r_column++){
				if(bitmap[row][r_column]!=r_prev && !r_prev){
					margin[row][RIGHT]=r_column;
					break;
				}
				r_prev = bitmap[row][r_column];
			}

			//check going out
			if(margin[row][LEFT]>MIDPOINT_REF){ // b w
				l_going_out++;
			}

			if (margin[row][RIGHT]<MIDPOINT_REF){ // w b
				r_going_out++;
			}
			midpoint[row] = (margin[row][LEFT]+margin[row][RIGHT])/2;
			//update data: white count within track
			if(!data[row][ISBLACK]){
				data[row][WHITECOUNT]=margin[row][RIGHT]-margin[row][LEFT];
			}
			else{
				data[row][WHITECOUNT]=0;
				midpoint[row] = MIDPOINT_REF;
			}

		}
	}

/************************COLLECT EVIDENCE***********************/

	//find black row (from top)
	double_check_black_end = CS;
	//if start from not black row-> find first black
	if(data[double_check_black_end][ISBLACK]==0){
		for(int black = CS; black<CE; black++){
			if(data[black][ISBLACK]==1){
				double_check_black_end = black;
				break;
			}
		}
	}
	//if start from black -> find last black
	else{
		for(int black = CS; black<CE; black++){
			if(data[black][ISBLACK]==0){
				double_check_black_end = black;
				break;
			}
		}
	}


	//find last black row (from bottom)
	black_end = CS;
	for(int black = CE-1; black>=CS; black--){
		if(data[black][ISBLACK]==1){
			black_end = black;
			break;
		}
	}

	//scan down from last black
	//find first white row
	white_start = black_end;
	for(int white = white_start; white<CE; white++){
		if(data[white][ISWHITE]==1){
			white_start = white;
			break;
		}
	}
	//find last white row
	white_end = white_start+1;
	for(int white = white_end; white<CE-1; white+=2){
		if(data[white][ISWHITE]==0){
			white_end = white-1; //last white row
			break;
		}
		white_end = white;
	}

	if(white_end>white_start)
	{
		white_count = white_end - white_start;
	}
	else
		white_count = 0;

	uint8_t checkrow = libutil::Clamp(CS,vip.highest_cont_margin-5,CE);

/************************DEFINE CASES***********************/

	//black guide line
	if(blp.detected()){
		bg = true;
	}

	else if(blp.approaching()){
		for(int i=blp.nearest_blackGuideLine; i>CS; i--){
			if(blp.margin[i][RIGHT]!=blp.margin[i][LEFT])
				midpoint[i]=blp.midpoint[i];
		}
	}


	//right angle
	else if(data[checkrow][ISBLACK]==1 && (vip.top_len>35 && vip.bottom_len>35)){
		//		black_line = false;
		right_angle = true;

		float h1 = HEIGHT-1;
		float h2 = black_end+2;
		float mid1 = midpoint[HEIGHT-1];
		float mid2 = midpoint[black_end+2];

		float slope = (h1-h2)/(mid1-mid2);

		if(slope<0.0f){
			RA_dir = true;
		}
	}

	//black line, keep until right angle
	else if(vip.top_len>35 && vip.bottom_len>35 && vip.bottom_len>vip.top_len){//&& abs(vip.bottom_len-vip.top_len)<30){

		black_line = true;

		uint8_t y1;
		uint8_t lx1;
		uint8_t rx1;

		uint8_t y2;
		uint8_t lx2;
		uint8_t rx2;

		y1 = vip.highest_cont_margin-10;
		lx1 = margin[y1][LEFT];
		rx1 = margin[y1][RIGHT];

		if(vip.lowest_cont_margin<CE-6){
			y2 = vip.lowest_cont_margin+5;
			lx2 = margin[y2][LEFT];
			rx2 = margin[y2][RIGHT];
		}
		else{
			y2 = vip.highest_cont_margin-5;
			lx2 = margin[y2][LEFT];
			rx2 = margin[y2][RIGHT];
		}

		float left_new_slope = (y1-y2)/(lx1-lx2);
		float right_new_slope = (y1-y2)/(rx1-rx2);

		for(int i = vip.highest_cont_margin; i< vip.lowest_cont_margin; i++){
			margin[i][LEFT] = libutil::Clamp((float)RS,1.0f/left_new_slope + margin[i-1][LEFT],(float)RE);
			margin[i][RIGHT] = libutil::Clamp((float)RS,1.0f/right_new_slope + margin[i-1][RIGHT],(float)RE);
		}

		for(int i= CS; i<CE; i++){
			midpoint[i] = (margin[i][LEFT]+margin[i][RIGHT])/2;
		}

	}

	/* keep angle until detected black line
	else if(right_angle){
		if(black_line){
			right_angle = false;
			continue_right_angle = false;
		}
		continue_right_angle = true;
	}

	*/

	//cross road: ensure there are few rows of white
	else if(white_count>HEIGHT/2){
//	else if(black_end<=15 && black_end>=4 && white_count >5){

		crossroad = true;

		uint8_t y1;
		uint8_t lx1;
		uint8_t rx1;

		uint8_t y2;
		uint8_t lx2;
		uint8_t rx2;

		uint8_t bias_count = 0;

		//define case: straight/ bias - evidence if one side lose margin = bias
		// bias: white->black->white

		if(white_start > black_end){


			for(int i = white_start-1; i>black_end+1; i--){

				bias_crossroad_margin[i][0] = RS;
				bias_crossroad_margin[i][1] = RE-1;
				bias_crossroad_margin[i][2] = RE-1;
				bias_crossroad_margin[i][3] = RS;

				if(data[i][ISBLACK]==0){

					/* 1. scan from left to right
					 * check start = white/black
					 * find first 2 changing point (margin) from left */

					bool prev = bitmap[i][RS];
					if(!prev){
						bias_crossroad_margin[i][0] = RS;
					}
					else{
						for(int j=RS; j<RE; j++){
							if(bitmap[i][j]!=prev && prev){

								bias_crossroad_margin[i][0] = j;
								break;
							}
							prev = bitmap[i][j];
						}
					}

					prev = bitmap[i][bias_crossroad_margin[i][0]+1];
					for(int j=bias_crossroad_margin[i][0]+1; j<RE; j++){
						if(bitmap[i][j]!=prev && !prev){
							bias_crossroad_margin[i][1] = j;
							break;
						}
						prev = bitmap[i][j];
					}

					/* 2. scan from right to left
					 * check start = white/black
					 * find first 2 changing point (margin) from right*/

					prev = bitmap[i][RE-1];
					if(!prev){
						bias_crossroad_margin[i][2] = RE-1;
					}
					else{
						for(int j=RE-1; j>=RS; j--){
							if(bitmap[i][j]!=prev && prev){

								bias_crossroad_margin[i][2] = j;
								break;
							}
							prev = bitmap[i][j];
						}

						prev = bitmap[i][bias_crossroad_margin[i][2]-1];
						for(int j=bias_crossroad_margin[i][2]-1; j>RS; j--){
							if(bitmap[i][j]!=prev && !prev){
								bias_crossroad_margin[i][3]=j;
								break;
							}
							prev = bitmap[i][j];
						}
					}

				}
				else{
					//if it's black row, follow last row margin
					bias_crossroad_margin[i][0] = bias_crossroad_margin[i+1][0];
					bias_crossroad_margin[i][1] = bias_crossroad_margin[i+1][1];
					bias_crossroad_margin[i][2] = bias_crossroad_margin[i+1][2];
					bias_crossroad_margin[i][3] = bias_crossroad_margin[i+1][3];
				}

				if(abs(bias_crossroad_margin[i][0]-bias_crossroad_margin[i][3])<=2 && abs(bias_crossroad_margin[i][1]-bias_crossroad_margin[i][2])<=2){
					bias_count++;
				}

			}

			//straight: most of the difference of (bias_crossroad_margin[i][0], bias_crossroad_margin[i][3]) & (bias_crossroad_margin[i][1],bias_crossroad_margin[i][2]) smaller than 2


			if(bias_count< (white_start-black_end)*2/3){
//				continue_bias_crossroad = false;

				y1 = black_end + (white_start - black_end)/2;
				lx1 = margin[y1][LEFT];
				rx1 = margin[y1][RIGHT];

				if(CE - white_end > 5){
					y2 = white_end + (CE-white_end)/2;
					lx2 = margin[y2][LEFT];
					rx2 = margin[y2][RIGHT];
				}
				else{
					y2 = white_start-5;
					lx2 = margin[y2][LEFT];
					rx2 = margin[y2][RIGHT];
				}

				float left_new_slope = (y1-y2)/(lx1-lx2);
				float right_new_slope = (y1-y2)/(rx1-rx2);

				for(int i = white_start-(white_start - black_end)/2; i< CE; i++){
					margin[i][LEFT] = libutil::Clamp((float)RS,1.0f/left_new_slope + margin[i-1][LEFT],(float)RE);
					margin[i][RIGHT] = libutil::Clamp((float)RS,1.0f/right_new_slope + margin[i-1][RIGHT],(float)RE);
				}

				for(int i= CS; i<CE; i++){
					midpoint[i] = (margin[i][LEFT]+margin[i][RIGHT])/2;
				}

			}

			//bias
			else{

				bias_crossroad = true;

				//get direction by servo degree
				//turning left
				if(car->GetServo().GetDegree()>=SERVO_MID_DEGREE){
					for(int i=black_end+1; i<white_start-1; i++){
						margin[i][0] = bias_crossroad_margin[i][0];
						margin[i][0] = bias_crossroad_margin[i][1];
					}

					y1 = black_end + (white_start - black_end)/2;
					rx1 = margin[y1][RIGHT];

					y2 = white_start-5;
					rx2 = margin[y2][RIGHT];

					float right_new_slope = (y1-y2)/(rx1-rx2);

					for(int i = white_start-(white_start - black_end)/2; i< CE; i++){
						margin[i][RIGHT] = libutil::Clamp((float)RS,1.0f/right_new_slope + margin[i-1][RIGHT],(float)RE);
					}

					for(int i= CS; i<CE; i++){
						midpoint[i] = (margin[i][LEFT]+margin[i][RIGHT])/2;
					}
				}

				//turning right
				else{
					for(int i=black_end+1; i<white_start-1; i++){
						margin[i][0] = bias_crossroad_margin[i][3];
						margin[i][0] = bias_crossroad_margin[i][2];
					}
					y1 = black_end + (white_start - black_end)/2;
					lx1 = margin[y1][LEFT];

					y2 = white_start-5;
					lx2 = margin[y2][LEFT];

					float left_new_slope = (y1-y2)/(lx1-lx2);

					for(int i = white_start-(white_start - black_end)/2; i< CE; i++){
						margin[i][LEFT] = libutil::Clamp((float)RS,1.0f/left_new_slope + margin[i-1][LEFT],(float)RE);
					}

					for(int i= CS; i<CE; i++){
						midpoint[i] = (margin[i][LEFT]+margin[i][RIGHT])/2;
					}
				}

			}

		}

	}

	//going out: going_out > threshold
	else if (l_going_out-black_end>CE*2/3){
		l_byebye = true;
	}
	else if(r_going_out-black_end>CE*2/3){
		r_byebye = true;
	}

	//to be safe
	else
	{
		right_angle = false;
		crossroad = false;
		bg = false;
	}
}


//calculate sum of midpoint in [start, end]
float ImageProcess::MidpointSumCal(uint16_t start, uint16_t end){
	float sum = 0;

	for(uint16_t k=start; k<end; k++){
		sum += midpoint[k];
	}

	return sum;
}


//decide what result to return
float ImageProcess::Analyze(void){

	float error = MIDPOINT_REF - MidpointSumCal(MS,ME)/10;

	if(bg){
		STATE = BLACK_GUIDE;
		prev_error = blp.Analyze(bitmap);
		return blp.Analyze(bitmap);
	}

	else if(r_byebye)
	{
		STATE = OUT_OF_BOUND;
		prev_error = 10000;
		return 10000;
	}

	else if(l_byebye)
	{
		STATE = OUT_OF_BOUND;
		prev_error = -10000;
		return -10000;
	}

	else if (right_angle){
		STATE = RIGHT_ANGLE;
		if(RA_dir){
			return -10000; //turn right
		}
		else{
			return 10000; //turn left
		}
	}

	else if (error>2||error<-2)
	{

		if(crossroad){
			STATE = CROSSROAD;
		}
		else{
			STATE = TURNING;
		}

		//*FACTOR as error is too small
		prev_error = error*FACTOR;
		return error*FACTOR;
	}

	else
	{
		STATE = STRAIGHT;
		prev_error = 0;
		return 0;
	}
}

void ImageProcess::printResult(){

		//Initiate LCD writer for printing real time information
		LcdTypewriter::Config writer_conf;
		writer_conf.lcd = &car->GetLcd();
		writer_conf.bg_color = libutil::GetRgb565(0x33, 0xB5, 0xE5);
		LcdTypewriter writer(writer_conf);

		//print filtered image
		for(uint16_t i=CS; i<CE; i++)
		{
			bool* ptr = bitmap[i];
			car->GetLcd().SetRegion(libsc::Lcd::Rect(0,i,WIDTH,1));
			car->GetLcd().FillBits(0,0xFFFF,ptr,WIDTH);
		}

		// print margin found
		for(uint16_t i=CS; i<CE; i++)
		{
			car->GetLcd().SetRegion({margin[i][0], i, 1, 1});
			car->GetLcd().FillColor(St7735r::kBlue);
			car->GetLcd().SetRegion({margin[i][1], i, 1, 1});
			car->GetLcd().FillColor(St7735r::kBlue);
		}

		for(uint16_t i=RS; i<RE; i++){
			car->GetLcd().SetRegion(libsc::Lcd::Rect(i, vip.margin[i][TOP], 1, 1));
			car->GetLcd().FillColor(St7735r::kYellow);
			car->GetLcd().SetRegion(libsc::Lcd::Rect(i, vip.margin[i][BOTTOM], 1, 1));
			car->GetLcd().FillColor(St7735r::kYellow);
		}

		for(uint8_t i=RS; i<RE; i++){
			if(i<=vip.top_cont_last){
				car->GetLcd().SetRegion(libsc::Lcd::Rect(vip.cont_top_margin[i][START_AT], vip.cont_top_margin[i][ROW_VALUE]-2, 1, 5));
				car->GetLcd().FillColor(St7735r::kRed);
			}

			if(i<=vip.bottom_cont_last){
				car->GetLcd().SetRegion(libsc::Lcd::Rect(vip.cont_bottom_margin[i][START_AT], vip.cont_bottom_margin[i][ROW_VALUE]-2, 1, 5));
				car->GetLcd().FillColor(St7735r::kRed);
			}
		}

//		for(uint16_t i=CS; i<CE; i++){
//
//			car->GetLcd().SetRegion({midpoint[i], i, 1, 1});
//			car->GetLcd().FillColor(St7735r::kRed);
//		}

		car->GetLcd().SetRegion(libsc::Lcd::Rect(0, 64, St7735r::GetW(), LcdTypewriter::GetFontH()));
		writer.WriteString(String::Format("%ld,%ld",vip.top_len,vip.bottom_len).c_str());

//		car->GetLcd().SetRegion(Lcd::Rect(0,blp.nearest_blackGuideLine,WIDTH,1));
//		car->GetLcd().FillColor(St7735r::kGreen);

		car->GetLcd().SetRegion(Lcd::Rect(0,vip.highest_cont_margin,WIDTH,1));
		car->GetLcd().FillColor(St7735r::kPurple);

		car->GetLcd().SetRegion(Lcd::Rect(0,vip.lowest_cont_margin,WIDTH,1));
		car->GetLcd().FillColor(St7735r::kGreen);

		car->GetLcd().SetRegion(Lcd::Rect(0, 80, St7735r::GetW(), LcdTypewriter::GetFontH()));
		if(right_angle){
			writer.WriteString("RRR");
		}
		else{
			writer.WriteString("!R!R!R");
		}

		car->GetLcd().SetRegion(Lcd::Rect(0, 96, St7735r::GetW(),LcdTypewriter::GetFontH()));
		if(RA_dir){
			writer.WriteString("to right");
		}
		else{
			writer.WriteString("to left");
		}

		car->GetLcd().SetRegion(Lcd::Rect(0,112, St7735r::GetW(), LcdTypewriter::GetFontH()));
		if(bg){
			writer.WriteString("BGBGBG");
		}
		else{
			writer.WriteString("!BG!BG!BG");
		}

//
//		car->GetLcd().SetRegion(Lcd::Rect(0, 128, St7735r::GetW(), LcdTypewriter::GetFontH()));
//		writer.WriteString(String::Format("%ld",vip.lowest_cont_margin).c_str());

	}

}




