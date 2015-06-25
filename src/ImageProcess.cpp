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
	slope(0),

	crossroad(false),
	l_byebye(false),
	r_byebye(false),
	right_angle(false),
	black_line(false),
	bg(false),
	continue_right_angle(false),
	bias_crossroad(false),
	continue_bias_crossroad(false),

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
	continue_right_angle=false;

	//for special settings
	black_line = false;
	bg = false;

	//filter and convert to bits
	medianFilter.medianFilter(image,bitmap);

	//analyze for black guide line at the same time
	blp.Analyze(bitmap);

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
			}

		}
	}

/************************COLLECT EVIDENCE***********************/

	//find last black row
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

	//check if it's black line before/after right angle
	//scan up from "last black +5"
	//stop when found normal row
	black_line = false;
	if(black_end>1)
		checkRA = black_end-1;
	else
		checkRA = CS;

	for(int white = checkRA; white>CS; white--){
		int count =0;

		for(int pixels=RS; pixels<WIDTH/2; pixels++){
			if(!bitmap[white][pixels])
				count++;
		}

		if(count>WIDTH/4){
			black_line = true;
			checkRA = white;
			break;
		}

		if(white==0){
			black_line = false;
		}
	}

	if(black_line && white_start>checkRA&& checkRA-black_end>CE/2){ // to prevent seeing other track // && checkRA-black_end>HEIGHT/2){
		//find last black row of first area
		black_line_start = checkRA;
		black_end = checkRA;
		for(int black = checkRA-1; black>CS; black--){
			int count=0;
			for(int pixels=RS; pixels<RE; pixels++){
				if(!bitmap[black][pixels])
					count++;
			}
			if(count==RE-5){
				black_end = black;
				break;
			}
			if(black==0){
				black_end=CS;
			}
		}

		for(int black = black_line_start+1; black<CE; black++){
			int count=0;
			for(int pixels=RS; pixels<RE; pixels++){
				if(!bitmap[black][pixels])
					count++;
			}
			if(count>WIDTH/4){
				black_line_end = black-1;
				break;
			}
			if(black==CE-1)
				black_line_end = CE-1;
		}

	}

	if(white_end>white_start)
	{
		white_count = white_end - white_start;
	}
	else
		white_count = 0;

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

	//keep angle until detected black line
	else if(right_angle){
		if(black_line){
			right_angle = false;
			continue_right_angle = false;
		}
		continue_right_angle = true;
	}

	//right angle: slope
	else if(black_end>CE/3 &&!black_line){

		float h1 = CE-1;
		float h2 = black_end+2;
		float mid1 = midpoint[CE-1];
		float mid2 = midpoint[black_end+2];

		slope = (h1-h2)/(mid1-mid2);

		if(abs(slope)>=3.7f){
//		if(abs(slope) >= 3.0f ){ //&& abs(slope)<5){
			right_angle=true;
		}

	}

	//cross road: ensure there are few rows of white
	else if(black_end<=15 && black_end>=4 && white_count >5){
		crossroad = true;

		uint8_t y1;
		uint8_t lx1;
		uint8_t rx1;

		uint8_t y2;
		uint8_t lx2;
		uint8_t rx2;

		//define case: straight/ bias - evidence if one side lose margin = bias
		uint8_t no_margin_count = 0;
		for(int i=CS; i<CE; i++){
			if(margin[i][LEFT]==RS || margin[i][RIGHT]==RE)
				no_margin_count++;
		}

		if(white_start > black_end){
			//bias
			if(no_margin_count>HEIGHT*5/6){

				bias_crossroad = true;

				//get direction by servo degree
				//turning left
				if(car->GetServo().GetDegree()>9500){

					for(int i = white_start-1; i>black_end+1; i--){
						if(data[i][ISBLACK]==0){

							//check start = white/black
							//find first 2 changing point (margin) from left
							bool prev = bitmap[i][RS];
							if(!prev){
								margin[i][LEFT] = RS;
							}
							else{
								for(int j=RS; j<RE; j++){
									if(bitmap[i][j]!=prev && prev){

											margin[i][LEFT] = j;
											break;
									}
									prev = bitmap[i][j];
								}
							}

							prev = bitmap[i][margin[i][LEFT]+1];
							for(int j=margin[i][LEFT]+1; j<RE; j++){
								if(bitmap[i][j]!=prev && !prev){
									margin[i][RIGHT] = j;
									break;
								}
								prev = bitmap[i][j];
							}

						}
						else{
							margin[i][LEFT] = margin[i+1][LEFT];
							margin[i][RIGHT] = margin[i+1][RIGHT];
						}

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

					for(int i = white_start-1; i>black_end+1; i--){
						if(data[i][ISBLACK]==0){

							//check start = white/black
							//find first 2 changing point (margin) from left
							bool prev = bitmap[i][RE-1];
							if(!prev){
								margin[i][RIGHT] = RE-1;
							}
							else{
								for(int j=RE-1; j>=RS; j--){
									if(bitmap[i][j]!=prev && prev){
											margin[i][RIGHT] = j;
											break;
									}
									prev = bitmap[i][j];
								}
							}

							prev = bitmap[i][margin[i][RIGHT]-1];
							for(int j=margin[i][RIGHT]-1; j>=RS; j--){
								if(bitmap[i][j]!=prev && !prev){
									margin[i][LEFT] = j;
									break;
								}
								prev = bitmap[i][j];
							}

						}
						else{
							margin[i][LEFT] = margin[i+1][LEFT];
							margin[i][RIGHT] = margin[i+1][RIGHT];
						}

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

			//straight
			else{

				continue_bias_crossroad = false;

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

	else if(continue_right_angle){
		STATE = RIGHT_ANGLE;
		if(prev_error>0)
			prev_error-=80;
		else
			prev_error+=80;
		return prev_error;

	}

	else if (right_angle){
		STATE = RIGHT_ANGLE;
		if(slope<0.0f){
			prev_error = -10000;
			return -10000; //turn right
		}
		else{
			prev_error = 10000;
			return 10000; //turn left
		}
	}

	else if (continue_bias_crossroad){
		return prev_error;
	}

	else if (error>2||error<-2)
	{
		STATE = TURNING;

		if(bias_crossroad){
			continue_bias_crossroad = true;
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

	//	car->GetLcd().SetRegion(libsc::Lcd::Rect(0,0,WIDTH,HEIGHT));
	//	car->GetLcd().FillColor(libsc::Lcd::kBlack);

		// print margin found
		for(uint16_t i=CS; i<CE; i++)
		{
			car->GetLcd().SetRegion({margin[i][0], i, 1, 1});
			car->GetLcd().FillColor(St7735r::kBlue);
			car->GetLcd().SetRegion({margin[i][1], i, 1, 1});
			car->GetLcd().FillColor(St7735r::kBlue);
		}
//
//		// print margin found
//		for(uint16_t i=CS; i<CE; i++){
//
//			car->GetLcd().SetRegion({blp.margin[i][0], i, 1, 1});
//			car->GetLcd().FillColor(St7735r::kYellow);
//			car->GetLcd().SetRegion({blp.margin[i][1], i, 1, 1});
//			car->GetLcd().FillColor(St7735r::kYellow);
//		}

//	 	 //print midpoint
//		for(uint16_t i=CS; i<CE; i++){
//				car->GetLcd().SetRegion({MIDPOINT_REF, i, 1, 1});
//				car->GetLcd().FillColor(St7735r::kCyan);
//		}

		for(uint16_t i=CS; i<CE; i++){

			car->GetLcd().SetRegion({midpoint[i], i, 1, 1});
			car->GetLcd().FillColor(St7735r::kRed);
		}

		//print Q and cross road related info

		car->GetLcd().SetRegion(libsc::Lcd::Rect(0,black_end,WIDTH,1));
		car->GetLcd().FillColor(St7735r::kGreen);

	//	car->GetLcd().SetRegion(libsc::Lcd::Rect(0,black_line_start,WIDTH,1));
	//	car->GetLcd().FillColor(St7735r::kGreen);
	//
	//	car->GetLcd().SetRegion(libsc::Lcd::Rect(0,black_line_end,WIDTH,1));
	//	car->GetLcd().FillColor(St7735r::kGreen);

		car->GetLcd().SetRegion(libsc::Lcd::Rect(0,white_start,WIDTH,1));
		car->GetLcd().FillColor(St7735r::kGreen);

		car->GetLcd().SetRegion(libsc::Lcd::Rect(0,white_end,WIDTH,1));
		car->GetLcd().FillColor(St7735r::kGreen);
	//
	//	car->GetLcd().SetRegion(libsc::Lcd::Rect(0,blp.nearest_blackGuideLine,WIDTH,1));
	//	car->GetLcd().FillColor(St7735r::kYellow);

		//BE,WS, WE for Q & crossroad
		car->GetLcd().SetRegion({0, 64, St7735r::GetW(), LcdTypewriter::GetFontH()});
		writer.WriteString(String::Format("%ld, %ld, %ld,%ld\n",black_end, checkRA, white_start, white_end).c_str());


		car->GetLcd().SetRegion({0, 80, St7735r::GetW(), LcdTypewriter::GetFontH()});
		writer.WriteString(String::Format("%ld",car->GetServo().GetDegree()).c_str());
//		if(blp.approaching()){
//			writer.WriteString(String::Format("Approaching: %ld",blp.nearest_blackGuideLine).c_str());
//		}
//		else
//			writer.WriteString(String::Format("!Approaching: %ld",blp.nearest_blackGuideLine).c_str());

		car->GetLcd().SetRegion({0, 96, St7735r::GetW(), LcdTypewriter::GetFontH()});
//		if(bg){
//			writer.WriteString(String::Format("BGBGBG: %ld",blp.narrow_count).c_str());
//		}
//		else
//			writer.WriteString(String::Format("!BG!BG!BG: %ld",blp.narrow_count).c_str());
		if(crossroad){
			writer.WriteString("XXX");
		}
		else{
			writer.WriteString("!X!X!X");
		}
//		if(black_line){
//			writer.WriteString("BLBLBL");
//		}
//		else{
//			writer.WriteString("!BL!BL!BL");
//		}

		car->GetLcd().SetRegion({0, 112, St7735r::GetW(), LcdTypewriter::GetFontH()});
		writer.WriteString(String::Format("%f",slope).c_str());
//		writer.WriteString(String::Format("%f",Analyze()).c_str());
//		if(right_angle){
//			writer.WriteString("RRR");
//		}
//		else{
//			writer.WriteString("!R!R!R");
//		}
//		if(white_start>black_end)
//			writer.WriteString(String::Format("%ld",blp.margin[black_end+(white_start-black_end)/2][1]-blp.margin[black_end+(white_start-black_end)/2][0]).c_str());

		car->GetLcd().SetRegion(libsc::Lcd::Rect(0,144, St7735r::GetW(),LcdTypewriter::GetFontH()));
	//	writer.WriteString(String::Format("%ld, %ld\n",black_line_start, black_line_end).c_str());
//		if(white_start>5)
//			writer.WriteString(String::Format("%ld, %ld\n",blp.margin[white_start-5][0], blp.margin[white_start-5][1]).c_str());
//		writer.WriteString(String::Format("%f",slope).c_str());
		writer.WriteString(String::Format("%f",Analyze()).c_str());

	}

}




