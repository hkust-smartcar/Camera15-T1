/*
 * blacklineProcess.cpp
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include "blacklineProcess.h"

namespace camera{
int16_t blacklineProcess::Analyze(bool bitmap[60][80]){

	for(int16_t row=HEIGHT-1; row>=0; row--){

		margin[row][LEFT]=RS;
		margin[row][RIGHT]=RE;
		//if midpoint = black, out of track already

		//start from previous midpoint
		bool l_prev = bitmap[row][RE];
		bool r_prev = bitmap[row][RE];

		// scan from left
		for(uint16_t l_column= RE/4; l_column<RE ; l_column++){
			if(bitmap[row][l_column]!=l_prev && !l_prev){
				margin[row][LEFT]=l_column;
				break;
			}
			l_prev = bitmap[row][l_column];
		}

		//scan from right
		for(uint16_t r_column= RE; r_column>0; r_column--){
			if(bitmap[row][r_column]!=r_prev && !r_prev){
				margin[row][RIGHT]=r_column;
				break;
			}
			r_prev = bitmap[row][r_column];
		}

		midpoint[row] = (margin[row][LEFT]+margin[row][RIGHT])/2;


	}

	double sum = 0;

	for(uint16_t k=35; k<45; k++){
		sum += midpoint[k];
	}

	return (37-sum/10)*100;

}

bool blacklineProcess::detected(){

	//detected = true when double line -> single line
	//evidence 1: number of rows with narrow width > threshold
	//evidence 2: no sudden change in midpoint of rows: current - prev > threshold

//	uint16_t prev_mid = midpoint[HEIGHT-1];
	narrow_count=0;

	for(int16_t row=HEIGHT-1; row>=0; row--){
		if(margin[row][RIGHT]-margin[row][LEFT] < 5){ // count narrow row
			narrow_count++;
		}
//		if(abs(midpoint[row]-prev_mid) > WIDTH/3){ // sudden change
//			return false;
//		}
//		prev_mid = midpoint[row];
	}

	if(narrow_count>45){ // if narrow row count > threshold
		return true;
	}

	return false;
}

}
