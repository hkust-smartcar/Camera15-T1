/*
 * distortion.h
 *
 *  Created on: 2015�~5��29��
 *      Author: Benlai
 */

#ifndef SRC_DISTORTION_H_
#define SRC_DISTORTION_H_

#include "libutil/math.h"
#include "libbase/misc_types.h"




class distortion
{
public:

//	int newx(int x,int y,int radius);
//
//	int newy(int x,int y,int radius);

	void correction(const Byte* image,bool AI[60][80]);


//	void correctionmatrix();




private:
//	int DI[60][80];
	int correctionRadius = 175;//51 to 500





};





#endif /* SRC_DISTORTION_H_ */
