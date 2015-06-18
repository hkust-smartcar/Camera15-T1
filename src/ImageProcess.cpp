/*
 * ImageProcess.cpp
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include "ImageProcess.h"
#include <algorithm>

namespace camera{

ImageProcess::ImageProcess()
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
	slope(CS),

	crossroad(false),
	l_byebye(false),
	r_byebye(false),
	right_angle(false),
	black_line(false),
	bg(false),

	STATE(0)
{
	for(int i=CS; i<CE; i++){
		midpoint[i] = MIDPOINT_REF;

		data[i][0]=0;
		data[i][1]=0;
		data[i][2]=0;
		data[i][3]=LEFT;
	}
}

void ImageProcess::start(Byte* image){

/************************INITIALIZATION***********************/

	//for cross road
	white_count = 0;
	crossroad = false;

	//for going out
	l_byebye = false;
	r_byebye = false;
	int l_going_out = 0;
	int r_going_out = 0;

	//for right angle
	right_angle = false;

	//for special settings
	black_line = false;
	bg = false;

	//filter and convert to bits
	medianFilter.medianFilter(image,bitmap);

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

	//right angle: slope
	else if(black_end>CE/4 &&!black_line){

		float h1 = CE-1;
		float h2 = black_end+2;
		float mid1 = midpoint[CE-1];
		float mid2 = midpoint[black_end+2];

		slope = (h1-h2)/(mid1-mid2);

		if(abs(slope)>3 && abs(slope)<5){
			right_angle=true;
		}

	}

	//cross road: ensure there are few rows of white
	else if(black_end<CE/6 && white_count >5){
		crossroad = true;
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
double ImageProcess::MidpointSumCal(uint16_t start, uint16_t end){
	double sum = 0;

	for(uint16_t k=start; k<end; k++){
		sum += midpoint[k];
	}

	return sum;
}


//decide what result to return
int ImageProcess::Analyze(void){

	double error = MIDPOINT_REF - MidpointSumCal(MS,ME)/10;

	if(bg){
		return blp.Analyze(bitmap);
	}

	else
		if(r_byebye)
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
		if(slope<0.0f){
			return -10000; //turn right
		}
		else{
			return 10000; //turn left
		}
	}

	else if (error>2||error<-2)
	{
		STATE = TURNING;
		//*FACTOR as error is too small
		return error*FACTOR;
	}

	else
	{
		STATE = STRAIGHT;
		return 0;
	}
}
}




