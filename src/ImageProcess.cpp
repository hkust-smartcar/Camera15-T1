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
	//evidence
	white_count(0),
	black_end(CS),
	checkRA(CS),
	white_start(CE-1),
	white_end(CE-1),

	//indicate situation
	crossroad(false),
	l_byebye(false),
	r_byebye(false),
	right_angle(false),
	black_line(false),
	black_guide_line(false),
	bias_crossroad(false),

	//indicate right angle direction
	RA_dir(false),

	STATE(0)
{
	for(int i=CS; i<CE; i++){
		//init midpoint array
		midpoint[i] = MIDPOINT_REF;

		//init data
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

	//for black line
	black_line = false;

	//for black guide line
	black_guide_line = false;

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

	uint8_t checkrow = vip.get_check_row();

/************************DEFINE CASES***********************/

	//black guide line
	if(blp.detected()){
		black_guide_line = true;
	}

	else if(blp.approaching()){
		for(int i=blp.get_nearest_blackGuideLine(); i>CS; i--){
			if(blp.get_margin(i,RIGHT)!=blp.get_margin(i,LEFT))
				midpoint[i]=blp.get_midpoint(i);
		}
	}


	//right angle
	else if(data[checkrow][ISBLACK]==1 && vip.detected_right_angle()){
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

	//black line
	else if(vip.detected_black_line()){

		black_line = true;

		uint8_t y1;
		uint8_t lx1;
		uint8_t rx1;

		uint8_t y2;
		uint8_t lx2;
		uint8_t rx2;

		y1 = vip.get_highest_cont_margin()-10;
		lx1 = margin[y1][LEFT];
		rx1 = margin[y1][RIGHT];

		if(vip.get_lowest_cont_margin()<CE-6){
			y2 = vip.get_lowest_cont_margin()+5;
			lx2 = margin[y2][LEFT];
			rx2 = margin[y2][RIGHT];
		}
		else{
			y2 = vip.get_highest_cont_margin()-5;
			lx2 = margin[y2][LEFT];
			rx2 = margin[y2][RIGHT];
		}

		float left_new_slope = (y1-y2)/(lx1-lx2);
		float right_new_slope = (y1-y2)/(rx1-rx2);

		for(int i = vip.get_highest_cont_margin(); i< vip.get_lowest_cont_margin(); i++){
			margin[i][LEFT] = libutil::Clamp((float)RS,1.0f/left_new_slope + margin[i-1][LEFT],(float)RE);
			margin[i][RIGHT] = libutil::Clamp((float)RS,1.0f/right_new_slope + margin[i-1][RIGHT],(float)RE);
		}

		for(int i= CS; i<CE; i++){
			midpoint[i] = (margin[i][LEFT]+margin[i][RIGHT])/2;
		}

	}

	//cross road: ensure there are few rows of white
	else if(white_count>HEIGHT/2){

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

			//straight: most of the difference of (bias_crossroad_margin[i][0], bias_crossroad_margin[i][3])
			// 			& (bias_crossroad_margin[i][1],bias_crossroad_margin[i][2]) smaller than 2
			if(bias_count< (white_start-black_end)*2/3){

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
		black_guide_line = false;
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

	if(black_guide_line){
		STATE = BLACK_GUIDE;
		return blp.Analyze(bitmap);
	}

	else if(r_byebye)
	{
		STATE = OUT_OF_BOUND;
		return 10000;
	}

	else if(l_byebye)
	{
		STATE = OUT_OF_BOUND;
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
		return error*FACTOR;
	}

	else
	{
		STATE = STRAIGHT;
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
			car->GetLcd().SetRegion(Lcd::Rect(0,i,WIDTH,1));
			car->GetLcd().FillBits(0,0xFFFF,ptr,WIDTH);
		}

		// print margin found
		for(uint16_t i=CS; i<CE; i++)
		{
			car->GetLcd().SetRegion(Lcd::Rect(margin[i][0], i, 1, 1));
			car->GetLcd().FillColor(St7735r::kBlue);
			car->GetLcd().SetRegion(Lcd::Rect(margin[i][1], i, 1, 1));
			car->GetLcd().FillColor(St7735r::kBlue);
		}

		//print midpoint found
		for(uint16_t i=CS; i<CE; i++){

			car->GetLcd().SetRegion(Lcd::Rect(midpoint[i], i, 1, 1));
			car->GetLcd().FillColor(St7735r::kRed);
		}

		//print state of car
		car->GetLcd().SetRegion(Lcd::Rect(0, 64, St7735r::GetW(), LcdTypewriter::GetFontH()));
		switch(STATE)
		{
		case INIT_STATE:
			writer.WriteString("INIT");
			break;

		case STRAIGHT:
			writer.WriteString("STRAIGHT");
			break;

		case CROSSROAD:
			writer.WriteString("CROSSROAD");
			break;

		case RIGHT_ANGLE:
			writer.WriteString("RIGHT ANGLE");
			break;

		case TURNING:
			writer.WriteString("TURN");
			break;

		case OUT_OF_BOUND:
			writer.WriteString("OUT");
			break;

		case BLACK_GUIDE:
			writer.WriteString("BLACK GUIDE");
			break;
		}


	}

}




