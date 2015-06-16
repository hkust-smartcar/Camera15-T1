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

	//find black line left to right
	for(int row=0; row<60; row++){
		bool prev = bitmap[row][0];

//		 scan to left
		for(int column= RS; column< RE; column++){
			if(bitmap[row][column]!=prev && prev){
				midpoint[row]= column;
				break;
			}
			prev = bitmap[row][column];
		}
	}

	return (37-midpoint[35])*100;

}
}
