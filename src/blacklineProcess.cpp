/*
 * blacklineProcess.cpp
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include "blacklineProcess.h"

namespace camera{
int16_t blacklineProcess::Analyze(bool bitmap[58][78]){

	nearest_blackGuideLine = 0;

	for(int16_t row=CE-1; row>=CS; row--){

		//initiation
		margin[row][LEFT]=RS;
		margin[row][RIGHT]=RE-1;

		bool l_prev = bitmap[row][RS];
		bool r_prev = bitmap[row][RE];

		// scan from left
		for(uint16_t l_column= RS; l_column<RE ; l_column++){
			if(bitmap[row][l_column]!=l_prev && !l_prev){
				margin[row][LEFT]=l_column;
				break;
			}
			l_prev = bitmap[row][l_column];
		}

		//scan from right
		for(uint16_t r_column= RE-1; r_column>RS; r_column--){
			if(bitmap[row][r_column]!=r_prev && !r_prev){
				margin[row][RIGHT]=r_column;
				break;
			}
			r_prev = bitmap[row][r_column];
		}

		midpoint[row] = (margin[row][LEFT]+margin[row][RIGHT])/2;

	}

	for(uint16_t j=CE-1; j>CS; j--){
		if(margin[j][RIGHT]-margin[j][LEFT]<5){
			nearest_blackGuideLine = j;
			break;
		}
	}

	double sum = 0;

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
	for(int16_t row=CE-1; row>CE-7; row--){
		if(margin[row][RIGHT]-margin[row][LEFT] < 5 && margin[row][RIGHT]-margin[row][LEFT] !=0){ // count narrow row
			narrow_count++;
			near ++;
		}
	}
	//other rows
	for(int16_t row=CE-7; row>=CS; row--){
		if(margin[row][RIGHT]-margin[row][LEFT] < 5 && margin[row][RIGHT]-margin[row][LEFT] !=0){ // count narrow row
			narrow_count++;
		}
	}

	if(narrow_count>15 && near>3){ //45){ // if narrow row count > threshold && right in front of car
		return true;
	}

	return false;
}

bool blacklineProcess::approaching(){

	//approaching = true when double line -> single line
	//evidence: nearest_blackGuideLine < threshold

	int count = 0;

	for(int16_t row=nearest_blackGuideLine; row>=HEIGHT/2; row--){
		if(margin[row][RIGHT]-margin[row][LEFT] < 5 && margin[row][RIGHT]-margin[row][LEFT] !=0){ // count narrow row
			count++;
		}
	}

	if(!detected() && nearest_blackGuideLine>CE-25 && count>10){
		return true;
	}
	return false;

}

}
