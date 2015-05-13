/*
 * ImageProcess.cpp
 *
 *  Created on: 4 Apr, 2015
 *      Author: Peggy
 */

#include "ImageProcess.h"

#define RS 5	//row start
#define RE 80-5	//row end

namespace camera{

ImageProcess::ImageProcess()
{
	for(int i=0; i<60; i++){
		midpoint[i] = MIDPOINT;
	}
}

int Median(bool* array){
	int sort[5];
	for(int i = 0; i < 5; i++){
		for(int j = 0; j < 5; j++){
			if(j > i){
				sort[i] = array[j] < array[i] ? array[j] : array[i];
			}
		}
	}
	return sort[3];
}


void ImageProcess::MedianFilter(bool* array_row, int length){
	bool* pRow = array_row;
	int data[length];
	for(int i = 0; i < length; i++){
		data[i] = Median(pRow++);
	}
	for(int i = 0; i < length; i++){
		array_row[i] = data[i];
	}

}

void ImageProcess::start(Byte* image){

	//initiation
	//for cross road
	white_count = 0;
	crossroad = false;
	start_row = 0;
	end_row = HEIGHT;
//	bool detected = false;

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

	//filter noise
	for(Uint filter_row=0; filter_row<HEIGHT; filter_row++){
		bool* array_ptr = bitmap[filter_row];
		MedianFilter(array_ptr,WIDTH);
	}

	// start image processing
	for(int16_t row=0; row<HEIGHT; row++){

		//start from previous midpoint
		margin[row][0] = RS;
		bool l_prev = bitmap[row][midpoint[row]];

		margin[row][1] = RS;
		bool r_prev = bitmap[row][midpoint[row]];

		// scan to left
		for(int l_column= midpoint[row]; l_column>RS ; l_column--){
			if(bitmap[row][l_column]!=l_prev && !l_prev){
				margin[row][0]=l_column;
				break;
			}
			l_prev = bitmap[row][l_column];
		}

		//scan to right
		for(int r_column=midpoint[row]; r_column<RE; r_column++){
			if(bitmap[row][r_column]!=r_prev && !r_prev){
				margin[row][1]=r_column;
				break;
			}
			r_prev = bitmap[row][r_column];
		}

		//if no right margin for this row
		if(margin[row][1]==RS){
			//if it's white (all pixels are false)
			if(!bitmap[row][RE-10]){
				margin[row][1]=RE;
			}

		}
		//check black row
		if(abs(margin[row][1]-margin[row][0])<10)
			black_count++;

		//check going out
		if(margin[row][0]>MIDPOINT){ // b w
			l_going_out++;
		}

		if (margin[row][1]<MIDPOINT){ // w b
			r_going_out++;
		}

		/* check for sudden change of edge
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
		 */
	}


//	//for row between suspected cross road
//	for(Uint r=start_row; r<end_row; r++){
//		//check white across width
//		if(abs(margin[r][1]-margin[r][0])>WIDTH-10)
//			white_count++;
//	}

	//check Q / cross road
	//find first white row
	//start from bottom (avoid noise at top)

	//if it start from white row, up until black line
	white_start = HEIGHT;
	if(abs(margin[white_start][1]-margin[white_start][0])>RE-10){
		while((abs(margin[white_start][1]-margin[white_start][0])>RE-5) && white_start<end_row){
			white_start--;
		}
	}
	//else it start from normal/black,up until white & up until black
	else{
		while(!(abs(margin[white_start][1]-margin[white_start][0])>RE-5) && white_start<end_row){
			white_start--;
		}
		while((abs(margin[white_start][1]-margin[white_start][0])>RE-5) && white_start<end_row){
			white_start--;
		}
	}
	white_end = white_start;
	while((abs(margin[white_end][1]-margin[white_end][0])>RE-10) && white_start<end_row){
		white_end++;
	}

//	int left_white=0;
//	int right_white=0;
	Qr = false;

	if(white_end>white_start)
	{

		for(Uint r=white_start; r<white_end; r++){
			//check white across width
			if(abs(margin[r][1]-margin[r][0])>WIDTH-10)
				white_count++;
		}
	}
	else
		white_count = 0;

	//find last black row
	black_end = 0;
	for(int black = white_start; black>0; black--){
		//count black pixels in row
		int black_pixel =0;
		for(int bp=0; bp<RE; bp++){
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
//	if(abs(black_end-white_start)<HEIGHT/8 || black_count>HEIGHT/3){
//		Q = true;
//	}
	if(black_end>HEIGHT/2){
		Q = true;
	}
	//ensure there are few rows of white
	else if(white_count >HEIGHT/12 && white_count <HEIGHT/2){
		crossroad = true;
	}
	//to be safe
	else
	{
		Q = false;
		crossroad = false;
	}

	if(Q){
		if(bitmap[black_end+1][RE]) //if right is black
			Qr = false;
		else
			Qr = true;
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
double ImageProcess::MidpointSumCal(Uint start, Uint end){
	double sum = 0;

	for(Uint k=start; k<end; k++){
		sum += midpoint[k];
	}

	return sum;
}

//decide what result to return
int ImageProcess::Analyze(void){

	double error = MIDPOINT - MidpointSumCal(28,38)/10;

	if(r_byebye)
		return 10000;
	else if(l_byebye)
		return -10000;
//	else if(black_end>HEIGHT/2){
//		return error*(FACTOR*1.2);
//	}
	else if (Q){
		if(Qr){
			return -10000; //turn right
		}
		else{
			return 10000; //turn left
		}
	}
	else if ((error>2||error<-2) && !crossroad)
		//*FACTOR as error is too small
		return error*FACTOR;
	else
		return 0;
}


}



