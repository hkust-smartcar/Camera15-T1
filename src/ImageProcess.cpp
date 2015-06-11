/*
 * ImageProcess.cpp
 *
 *  Created on: 4 Apr, 2015
 *      Author: Peggy
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
{
	for(int i=0; i<60; i++){
		midpoint[i] = MIDPOINT;

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
	start_row = 0;
	end_row = HEIGHT;

	//for going out
	l_byebye = false;
	r_byebye = false;
	int l_going_out = 0;
	int r_going_out = 0;

	//for right angle
	right_angle = false;
	Qr = false;

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
	for(int16_t row=HEIGHT; row>0; row--){

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
			if(!bitmap[row][i]) //if black
				lwc++;
		}
		for(int i = RE/2; i<RE; i++){
			if(!bitmap[row][i]) //if black
				rwc++;
		}
		if(rwc>lwc){
			data[row][WHITEAT]=RIGHT;
		}

		// avoid black midpoint
		int16_t mid;

		if(row == HEIGHT)
		{

			if(data[row][ISBLACK]==1){
				while(row>0 && data[row][ISBLACK]==0){
					margin[row][LEFT]=RS;
					margin[row][RIGHT]=RS;
					midpoint[row]=RS;
					row--;
				}
			}
			mid = midpoint[row];

			//check if prev midpoint is black
			if(bitmap[row][mid]){
				if(data[row][WHITEAT]==LEFT){
					for(int i=mid; i>RS; i--){
						if(bitmap[row][mid]){
							mid--;
						}
						else{
							break;
						}
					}
				}
				else{
					for(int i=mid; i<RE; i++){
						if(bitmap[row][mid]){
							mid++;
						}
						else{
							break;
						}
					}
				}
			}
		}

		//prevent sudden change of midpoint
		else{
			mid = midpoint[row+1];
		}

		margin[row][LEFT]=RS;
		margin[row][RIGHT]=RS;
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

			//if no right margin for this row
			if(margin[row][RIGHT]==RS){
				//if it's white (all pixels are false)
				if(!bitmap[row][RE-10]){
					margin[row][RIGHT]=RE;
				}

			}
		}
		//if it's suppose to be black
		else{
			for(int sth=row; sth>0; sth--){
					data[sth][WHITECOUNT]=0;
					data[sth][ISBLACK]=1;
					data[sth][ISWHITE]=0;
					midpoint[sth]=0;
					r_going_out++;
			}
			break;

		}
		//check going out
		if(margin[row][LEFT]>MIDPOINT){ // b w
			l_going_out++;
		}

		if (margin[row][RIGHT]<MIDPOINT){ // w b
			r_going_out++;
		}

		midpoint[row] = (margin[row][LEFT]+margin[row][RIGHT])/2;
		//update data: white count within track
		data[row][WHITECOUNT]=margin[row][RIGHT]-margin[row][LEFT];

	}

	//check right angle/ cross road

		//find last black row
		black_end = 0;
		for(int black = HEIGHT; black>0; black--){
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
		for(int white = white_end; white<HEIGHT; white+=2){
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
		checkRA =black_end+1;

		for(int white = HEIGHT; white>0; white--){
			int count =0;
			if(data[black_end][WHITEAT]==LEFT){
				for(int pixels=0; pixels<WIDTH; pixels++){
					if(!bitmap[white][pixels])
						count++;
				}
			}
			else{
				for(int pixels=0; pixels<WIDTH; pixels++){
					if(!bitmap[white][pixels])
						count++;
				}
			}
			if(count>WIDTH/4){
				black_line = true;
				checkRA = white;
				break;
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
				if(black==HEIGHT)
					black_line_end = HEIGHT;
			}
			//get those margin back!!!

		}

		if(white_end>white_start)
		{
			white_count = white_end - white_start;
		}
		else
			white_count = 0;

		//if distance between last black row & first white row < threshold, consider as Q
		// or black count > threshold, consider as Q

		if(black_end>HEIGHT/4 &&!black_line){ //right angle?
			int prev_white_count = data[black_end+1][WHITECOUNT];
			for(int i=black_end+2; i<HEIGHT; i++){
				if(abs(data[i][WHITECOUNT]-prev_white_count)>WIDTH/3 && data[black_end+HEIGHT/12][WHITECOUNT]>WIDTH/2){
					right_angle=true;
					break;
				}
			}
		}
		//ensure there are few rows of white
		else if(black_end<HEIGHT/12 && white_count >5){
			crossroad = true;
		}
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
			Qr=false;
			right_angle = false;
			crossroad = false;
		}

		if(right_angle){

			if(data[black_end+1][WHITEAT]==LEFT) //if right is black
				Qr = false;
			else
				Qr = true;
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

	double error = MIDPOINT - MidpointSumCal(CS,CE)/10;

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
		if(Qr){
			return -10000; //turn right
		}
		else{
			return 10000; //turn left
		}
//		return error*FACTOR;
	}
	else if (crossroad)
	{
		STATE = CROSSROAD;
		return (MIDPOINT - MidpointSumCal(libutil::Clamp(0,(int)white_start-1,60),libutil::Clamp(0,(int)white_start-11,60)))*FACTOR/10;
	}
//	else if(black_line){
////		return 0;
////		return (MIDPOINT - MidpointSumCal(libutil::Clamp(0,(int)black_line_end+1,60),libutil::Clamp(0,(int)black_line_end+11,60))/10)*FACTOR;
////		if(white_end-white_start>10){
////			return (MIDPOINT - MidpointSumCal(libutil::Clamp(0,(int)white_start,60),libutil::Clamp(0,(int)white_start+10,60))/10)*FACTOR;
////		}
////		if((black_line_start>(CS-20)&&black_line_start<(CE+20))||(black_line_end>(CS-20)&&black_line_end<(CE+20))){
////			return 0;
////		}
//		if(black_line_end>CS-15){
//			return 0;
//		}
//		else{
//			return error*FACTOR;
//		}
//	}
	else if ((error>2||error<-2) && !crossroad)
	{
		STATE = TURNING;
		//*FACTOR as error is too small
		return error*FACTOR;
	}
//	else if(black_end>HEIGHT*5/6){
////		return -error*10000;
//		return 0;
//	}
	else
	{
		STATE = STRAIGHT;
		return 0;
	}
}


}



