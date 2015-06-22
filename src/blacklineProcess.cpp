/*
 * blacklineProcess.cpp
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include "blacklineProcess.h"

namespace camera{
float blacklineProcess::Analyze(bool bitmap[58][78]){

	nearest_blackGuideLine = 0;

	for(int16_t row=CE-1; row>=CS; row--){

		//ensure start from white
		int start = RS;
		int end = RE-1;

		while(bitmap[row][start] && start<end){start++;}
		while(bitmap[row][end] && end>start){end--;}

		//initiation
		margin[row][LEFT]=start;
		margin[row][RIGHT]=end;

		bool l_prev = bitmap[row][start];
		bool r_prev = bitmap[row][end];

		// scan from left
		for(uint16_t l_column= start; l_column<end ; l_column++){
			if(bitmap[row][l_column]!=l_prev && !l_prev){
				margin[row][LEFT]=l_column;
				break;
			}
			l_prev = bitmap[row][l_column];
		}

		//scan from right
		for(uint16_t r_column= end; r_column>start; r_column--){
			if(bitmap[row][r_column]!=r_prev && !r_prev){
				margin[row][RIGHT]=r_column;
				break;
			}
			r_prev = bitmap[row][r_column];
		}

		midpoint[row] = (margin[row][LEFT]+margin[row][RIGHT])/2;

	}

	// if not white->black->white, filter out data
	for(uint16_t i=CE-1; i>CS; i--){

		uint16_t check1 = libutil::Clamp(CS,margin[i][LEFT]-5,CE-1);
		uint16_t check2 = libutil::Clamp(CS,margin[i][RIGHT]+5,CE-1);

		if(margin[i][LEFT]==RS || margin[i][RIGHT]==RE || bitmap[i][check1] || bitmap[i][check2] || margin[i][LEFT] > margin[i][RIGHT] || margin[i][RIGHT] < margin[i][LEFT]){
			margin[i][RIGHT]=margin[i][LEFT];
		}
	}

	for(int j=CE-1; j>CS; j--){
		if(margin[j][RIGHT]-margin[j][LEFT]<5 && margin[j][RIGHT]-margin[j][LEFT] !=0){
			nearest_blackGuideLine = j;
			break;
		}
	}


	float sum = 0;

	for(uint16_t k=MS; k<ME; k++){
		sum += midpoint[k];
	}

	return (MIDPOINT_REF-sum/10)*100;

}

bool blacklineProcess::detected(){

	//detected = true when arrive single line zone
	//evidence 1: number of rows with narrow width > threshold
	//evidence 2: black line start near car

	int near = 0;
	narrow_count=0;

	//nearest 5 rows
	for(int16_t row=CE-1; row>CE-6; row--){
		if(margin[row][RIGHT]-margin[row][LEFT] < 6 && margin[row][RIGHT]-margin[row][LEFT] !=0){ // count narrow row
			narrow_count++;
			near ++;
		}
	}
	//other rows
	for(int16_t row=CE-6; row>=CS; row--){
		if(margin[row][RIGHT]-margin[row][LEFT] < 5 && margin[row][RIGHT]-margin[row][LEFT] !=0){ // count narrow row
			narrow_count++;
		}
	}

	if(narrow_count>30 && near>3){ //45){ // if narrow row count > threshold && right in front of car
		return true;
	}

	return false;
}

bool blacklineProcess::approaching(){

	//approaching = true when double line -> single line
	//evidence: nearest_blackGuideLine < threshold

	int count = 0;

	for(int16_t row=nearest_blackGuideLine; row>CS; row--){
		if(margin[row][RIGHT]-margin[row][LEFT] < 5 && margin[row][RIGHT]-margin[row][LEFT] !=0){ // count narrow row
			count++;
		}
	}

	if(!detected() && nearest_blackGuideLine>CE-25 && count>20){
		return true;
	}
	return false;

}

}
