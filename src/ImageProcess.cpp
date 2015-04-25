/*
 * ImageProcess.cpp
 *
 *  Created on: 4 Apr, 2015
 *      Author: Peggy
 */

#include "ImageProcess.h"

#define HEIGHT 60
#define WIDTH 80

namespace camera{

ImageProcess::ImageProcess()
{
	for(int i=0; i<60; i++){
		midpoint[i] = MIDPOINT;
	}
}

void ImageProcess::start(Byte* image){

	//initiation
	//for cross road
	white_count = 0;
	crossroad = false;
	int start_row = 0;
	int end_row = WIDTH;
	bool detected = false;
	//for Q
	Q = false;
	black_count = 0;
	//for going out
	l_byebye = false;
	r_byebye = false;
	int l_going_out = 0;
	int r_going_out = 0;

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
	for(int16_t row=0; row<HEIGHT; row++){

		margin[row][0] = 0;
		bool l_prev = bitmap[row][midpoint[row]];

		margin[row][1] = 0;
		bool r_prev = bitmap[row][midpoint[row]];

		for(int l_column= midpoint[row]; l_column>0 ; l_column--){
			if(bitmap[row][l_column]!=l_prev && !l_prev){
				margin[row][0]=l_column;
				break;
			}
			l_prev = bitmap[row][l_column];
		}
		for(int r_column=midpoint[row]; r_column<WIDTH; r_column++){
			if(bitmap[row][r_column]!=r_prev && !r_prev){
				margin[row][1]=r_column;
				break;
			}
			r_prev = bitmap[row][r_column];
		}
		//if no right margin for this row
		if(margin[row][1]==0){
			//if it's white (all pixels are false)
			if(!bitmap[row][WIDTH-10]){
				margin[row][1]=WIDTH;
			}

		}
		//check black row
		if(abs(margin[row][1]-margin[row][0])<10)
			black_count++;

		//check going out
		if(margin[row][0]>MIDPOINT){
			l_going_out++;
		}

		if (margin[row][1]<MIDPOINT){
			r_going_out++;
		}

		// check for sudden change of edge
		if(row>3 && (abs(margin[row][0]-margin[row-3][0])>WIDTH/6 && abs(margin[row][1]-margin[row-3][1])>WIDTH/6)){
			// mark start & end
			if(!detected){
				start_row = row;
				detected = true;
			}
			else{
				end_row = row;
				detected = false;
			}
		}
		if(end_row<start_row){
			end_row = HEIGHT;
		}

	}

	//for row between suspected cross road
	for(int r=start_row; r<end_row; r++){
		//check white across width
		if(abs(margin[r][1]-margin[r][0])>WIDTH-10)
			white_count++;
	}

	//check Q
	//find first white row
	int white_start = start_row;
	while(!(abs(margin[white_start][1]-margin[white_start][0])>WIDTH-10) && white_start<end_row){
		white_start++;
	}
	//find last black row
	int black_end = 0;
	for(int black = white_start; black>0; black--){
		//count black pixels in row
		int black_pixel =0;
		for(int bp=0; bp<WIDTH; bp++){
			if(bitmap[black][bp])
				black_pixel++;
		}
		//if black > threshold, consider as black row
		if(black_pixel>WIDTH*3/4){
			black_end = black;
			break;
		}
	}
	//if distance between last black row & first white row < threshold, consider as Q
	// or black count > threshold, consider as Q
	if(abs(black_end-white_start)<HEIGHT/8 || black_count>HEIGHT/3){
		Q = true;
	}
	//ensure there are few rows of white
	else if(white_count >HEIGHT/12 && white_count <HEIGHT/2){
		crossroad = true;
	}

	//going out if going_out > threshold
	if (l_going_out>HEIGHT*2/3){
		l_byebye = true;
	}
	if(r_going_out>HEIGHT*2/3){
		r_byebye = true;
	}

	//calculate midpoint
	for(int k=0; k<HEIGHT; k++){
		midpoint[k] = (margin[k][0]+margin[k][1])/2;
	}

}

//calculate sum of midpoint in [start, end]
double ImageProcess::MidpointSumCal(int start, int end){
	double sum = 0;

	for(int k=start; k<end; k++){
		sum += midpoint[k];
	}

	return sum;
}

//For larger error, multiply larger FACTOR, vice versa
double MultiplyRatio(double err, int FACTOR){

	if(err>10 || err<-10){
		return err*(FACTOR*1.3);
	}
	else if (err>5 || err <-5){
		return err*(FACTOR*0.8);
	}
	else{
		return err*FACTOR;
	}
}

//decide what result to return
int ImageProcess::Analyze(void){

	double error = MIDPOINT - MidpointSumCal(28,38)/10;

	if(r_byebye)
		return 10000;
	else if(l_byebye)
		return -10000;
	else if ((error>2||error<-2) && !crossroad)
		return error;
	else
		return 0;
}


}



