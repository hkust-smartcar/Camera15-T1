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

int Median(bool* array){
	int sort[9];
	for(int i = 0; i < 9; i++){
		for(int j = 0; j < 9; j++){
			if(j > i){
				sort[i] = array[j] < array[i] ? array[j] : array[i];
			}
		}
	}
	return sort[5];
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

void ImageProcess::start(Byte* image, JyMcuBt106* bt){
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
	//	//filter noise
	//	for(Uint filter_row=0; filter_row<HEIGHT; filter_row++){
	//
	//		bool* array_ptr = bitmap[filter_row];
	//		MedianFilter(array_ptr,WIDTH);
	//	}

	// start image processing
	// go through each row, if find all white, guess its margin
	int l_difference = 0;
	int r_difference = 0;

	int mark =0; // mark last row with no data to compare with

	for(int16_t row=0; row<HEIGHT; row++){
		margin[row][0] = 5;
		bool l_prev = bitmap[row][midpoint[row]];

		margin[row][1] = 5;
		bool r_prev = bitmap[row][midpoint[row]];

		for(int l_column= midpoint[row]; l_column>5 ; l_column--){
			if(bitmap[row][l_column]!=l_prev && !l_prev){
				margin[row][0]=l_column;
				break;
			}
			l_prev = bitmap[row][l_column];
		}
		for(int r_column=midpoint[row]; r_column<WIDTH-5; r_column++){
			if(bitmap[row][r_column]!=r_prev && !r_prev){
				margin[row][1]=r_column;
				break;
			}
			r_prev = bitmap[row][r_column];
		}
		//if no right margin for this row
		if(margin[row][1]==5){
			//if it's white (all pixels are false)
			if(!bitmap[row][WIDTH-10])
				margin[row][1]=WIDTH-5;
		}

	}

	for(int k=0; k<HEIGHT; k++){
		midpoint[k] = (margin[k][0]+margin[k][1])/2;
	}

	//	// find potential white bar from center to top & bottom
	//	int normal_row=HEIGHT/6;
	//	while(margin[normal_row][1]-margin[normal_row][0]<WIDTH/2 && normal_row<WIDTH){
	//		normal_row++;
	//	}
	//	int l_slope = abs(margin[normal_row+1][0] - margin[normal_row][0]);
	//	int r_slope = abs(margin[normal_row+1][1] - margin[normal_row][1]);
	//
	//	for(int i= HEIGHT/2; i>0; i--){
	//
	//		if (margin[i+1][1]-margin[i][0]>WIDTH/2){
	//			if(margin[i+1][0]-l_slope>5)
	//				margin[i][0]=margin[i+1][0]+l_slope;
	//			else
	//				margin[i][0]=5;
	//			if(margin[i+1][1]+r_slope<WIDTH-5)
	//				margin[i][1]=margin[i+1][1]-r_slope;
	//			else
	//				margin[i][1]= WIDTH-5;
	//		}
	//	}
	//	for(int j= HEIGHT/2; j<HEIGHT; j++){
	//
	//		if (margin[j][1]-margin[j][0]>WIDTH/2){
	//			if(margin[j-1][0]+l_slope>5)
	//				margin[j][0]=margin[j-1][0]-l_slope;
	//			else
	//				margin[j][0]=margin[j-1][0]=5;
	//			if(margin[j+1][1]-r_slope<WIDTH-5)
	//				margin[j][1]=margin[j-1][1]+r_slope;
	//			else
	//				margin[j][1]=WIDTH-5;
	//
	//		}
	//
	//	}
	//





	//
	//		//calculate x difference between rows
	//		if(row <= 1){
	//			l_difference = 0;
	//			r_difference = 0;
	//		}
	//		else {
	//			l_difference = margin[row-1][0]-margin[row-2][0];
	//			r_difference = margin[row-1][1]-margin[row-2][1];
	//		}
	//
	//		//if its width > threshold -> cross
	//		if(margin[row][1]-margin[row][0]>WIDTH/3*2){
	//			//record row that can't be compare
	////			if(l_difference==0 && r_difference==0){
	////				mark = row;
	////			}
	//			//guess margin
	//			if (row>0){
	//				if(margin[row-1][0]-l_difference>5)
	//					margin[row][0]=margin[row-1][0]-l_difference;
	//				else
	//					margin[row][0] = 5;
	//
	//				if(margin[row-1][1]+r_difference<WIDTH-5)
	//					margin[row][1]=margin[row-1][1]+r_difference;
	//				else
	//					margin[row][1] = WIDTH-5;
	//			}
	//
	//		char buffer[100];
	//			sprintf(buffer,"left margin at row %d = %d\n right margin at row %d = %d\n",row,margin[row][0],row,margin[row][1]);
	//			bt->SendStr(buffer);
	//
	//	}// for row
	//
	//	//double check for white row
	//	if(mark>0){
	//		for(int x=mark; x>0; x--){
	//			l_difference = (abs(margin[x+5][0] - margin[x+4][0])+(margin[x+4][0] - margin[x+3][0])+(margin[x+3][0] - margin[x+2][0])+(margin[x+2][0] - margin[x+1][0]))/4;
	//			r_difference = (abs(margin[x+5][1] - margin[x+4][1])+(margin[x+4][1] - margin[x+3][1])+(margin[x+3][1] - margin[x+2][1])+(margin[x+2][1] - margin[x+1][1]))/4;
	//
	//			if(margin[x-1][0]-l_difference>5)
	//				margin[x][0]=margin[x-1][0]-l_difference;
	//			else
	//				margin[x][0] = 5;
	//
	//			if(margin[x-1][1]+r_difference<WIDTH-5)
	//				margin[x][1]=margin[x-1][1]+r_difference;
	//			else
	//				margin[x][1] = WIDTH-5;
	//		}
	//	}
	//	black_until=0;
	//	//filter out nearby tracks
	//	for(int y=0; y<HEIGHT; y++){
	//		//if it's black row
	//		if(margin[y][1]==5)
	//			black_until = y;
	//	}
	//	//line before last black line become black
	//	for(int turn_bk=black_until; turn_bk>0; turn_bk--){
	//		margin[turn_bk][0] = 5;
	//		margin[turn_bk][1] = 5;
	//	}
}

}


