/*
 * ImageProcess.cpp
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include "ImageProcess.h"
#include <algorithm>

#define RS 0	//row start
#define RE 80	//row end
#define WHITECOUNT 0
#define ISWHITE 1
#define ISBLACK 2
#define WHITEAT 3
#define CS 35
#define CE 45
#define LEFT 0
#define RIGHT 1

namespace camera{

ImageProcess::ImageProcess()
:
	black_count(0),
	white_count(0),

	FACTOR(100),

	black_end(0),
	checkRA(0),
	white_start(HEIGHT-1),
	white_end(HEIGHT-1),
	black_line_start(0),
	black_line_end(0),
	slope(0),

	crossroad(false),
	l_byebye(false),
	r_byebye(false),
	right_angle(false),
	black_line(false),
	bg(false),

	STATE(0)
{
	for(int i=0; i<60; i++){
		midpoint[i] = MIDPOINT_REF;

		data[i][0]=0;
		data[i][1]=0;
		data[i][2]=0;
		data[i][3]=LEFT;
	}
}

void ImageProcess::start(Byte* image){

	//initiation
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


	//	dis.correction(image,bitmap);
	//convert to bits
	for(int16_t image_row=0; image_row<HEIGHT; image_row++){
		for(int16_t byte=0; byte<10; byte++){
			Byte check_image = image[image_row*10+byte];
			/*to bit*/
			for(int16_t bit=7; bit>=0; bit--){
				bitmap[image_row][8*byte+bit] = check_image & 0x01;
				check_image >>= 1;
			}
		}
	}

	// start image processing
	for(int16_t row=HEIGHT-1; row>=0; row--){

		data[row][WHITECOUNT]= 0;
		data[row][ISWHITE] = 0;
		data[row][ISBLACK] = 0;
		data[row][WHITEAT] = LEFT;

		//collect row data: white pixels in row
		for(int pixels=0; pixels<WIDTH; pixels++){
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

		if(row == HEIGHT-1){
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
			for(int l_column= mid; l_column>RS ; l_column--){
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

	//check right angle/ cross road

	//find last black row
	black_end = 0;
	for(int black = HEIGHT-1; black>=0; black--){
		if(data[black][ISBLACK]==1){
			black_end = black;
			break;
		}
	}

	//scan down from last black
	//find first white row
	white_start = black_end;
	for(int white = white_start; white<HEIGHT; white++){
		if(data[white][ISWHITE]==1){
			white_start = white;
			break;
		}
	}
	//find last white row
	white_end = white_start+1;
	for(int white = white_end; white<HEIGHT-1; white+=2){
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
		checkRA = 0;

	for(int white = checkRA; white>0; white--){
		int count =0;

		for(int pixels=0; pixels<WIDTH/2; pixels++){
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

	if(black_line && white_start>checkRA&& checkRA-black_end>HEIGHT/2){ // to prevent seeing other track // && checkRA-black_end>HEIGHT/2){
		//find last black row of first area
		black_line_start = checkRA;
		black_end = checkRA;
		for(int black = checkRA-1; black>0; black--){
			int count=0;
			for(int pixels=0; pixels<WIDTH; pixels++){
				if(!bitmap[black][pixels])
					count++;
			}
			if(count==WIDTH-5){
				black_end = black;
				break;
			}
			if(black==0){
				black_end=0;
			}
		}

		for(int black = black_line_start+1; black<HEIGHT; black++){
			int count=0;
			for(int pixels=0; pixels<WIDTH; pixels++){
				if(!bitmap[black][pixels])
					count++;
			}
			if(count>WIDTH/4){
				black_line_end = black-1;
				break;
			}
			if(black==HEIGHT-1)
				black_line_end = HEIGHT-1;
		}

	}

	if(white_end>white_start)
	{
		white_count = white_end - white_start;
	}
	else
		white_count = 0;

	//for black guide line
	//find range of "almost" white row
//	almost_white_number = 0;
//	int check_nearly_white_margin = 0;
//	for(int white = HEIGHT/2; white>0; white--){
//		uint16_t wc = 0;
//		for(int c=0; c<RE; c++){
//			if(!bitmap[white][c])
//				wc++;
//		}
//		if(wc>WIDTH-20 && data[white][ISWHITE] == 0){
//			almost_white_number++;
//		}
//	}
//	for(int white = HEIGHT/2; white<HEIGHT; white++){
//		if(margin[white][0]-MIDPOINT_REF<5 || margin[white][1]-MIDPOINT_REF<5){
//			check_nearly_white_margin++;
//		}
//	}
//	for(int white = HEIGHT-1; white>HEIGHT/2; white--){
//			if(data[white][WHITECOUNT]<WIDTH/2){
//				almost_white_number++;
//			}
//		}

	if(blp.detected()){
		bg = true;
	}
	else if(black_end>HEIGHT/4 &&!black_line){ //right angle?

		float h1 = HEIGHT-1;
		float h2 = black_end+2;
		float mid1 = midpoint[HEIGHT-1];
		float mid2 = midpoint[black_end+2];

		slope = (h1-h2)/(mid1-mid2);

		if(abs(slope)>3 && abs(slope)<5){
			right_angle=true;
		}

	}
	//ensure there are few rows of white
	else if(black_end<HEIGHT/6 && white_count >5){
		crossroad = true;
	}
	//black guide line?
//	else if(black_end<HEIGHT/4 && almost_white_number > 5 && check_nearly_white_margin>HEIGHT/3){
	//going out if going_out > threshold
	else if (l_going_out-black_end>HEIGHT*2/3){
		l_byebye = true;
	}
	else if(r_going_out-black_end>HEIGHT*2/3){
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

	double error = MIDPOINT_REF - MidpointSumCal(CS,CE)/10;

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




