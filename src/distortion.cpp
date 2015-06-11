/*
 * distortion.cpp
 *
 *  Created on: 2015�~5��29��
 *      Author: Benlai
 */

#include "distortion.h"

/*
int distortion::newx(int x,int y,int radius)
{
	float theta = 0;
	int newx,newy;

	int imageWidth = 80;
	int imageHeight = 60;

	int halfWidth = imageWidth / 2;
	int halfHeight = imageHeight / 2;

	newx = x - halfWidth;
	newy = y - halfHeight;
	float distance = libutil::Math::Sqrt2(newx * newx + newy * newy);
	float r = float(distance /radius);
	if(r<1)
	{
		theta = 1-r;
	}
	else
	{
		theta = 1;
	}

	float xx = theta *newx;
	//float yy = theta *newy;
	int sourcex = halfWidth + xx;
	//int sourcey = halfHeight + yy;

	return sourcex;
}



int distortion::newy(int x, int y,int radius)
{
	float theta = 0;
	int newx,newy;

	int imageWidth = 80;
	int imageHeight = 60;

	int halfWidth = imageWidth / 2;
	int halfHeight = imageHeight / 2;

	newx = x - halfWidth;
	newy = y - halfHeight;
	float distance = libutil::Math::Sqrt2(newx * newx + newy * newy);
	float r = float(distance /radius);
	if(r<1)
	{
		theta = 1-r;
	}
	else
	{
		theta = 1;
	}

	//float xx = theta *newx;
	float yy = theta *newy;
	//int sourcex = halfWidth + xx;
	int sourcey = halfHeight + yy;

	return sourcey;

}

*/

void distortion::correction(const Byte* image,bool AI[60][80])
{

	float theta = 0;
	int newx,newy;

	int imageWidth = 80;
	int imageHeight = 60;

	int halfWidth = imageWidth / 2;
	int halfHeight = imageHeight / 2;

	for(int i = 0; i<60; i++)
	{

		for(int k =0 ; k<80; k++)
		{

			newx = k - halfWidth;
			newy = i - halfHeight;
			float distance = libutil::Math::Sqrt2(newx * newx + newy * newy);
			float r = float(distance /correctionRadius);
			if(r<1)
			{
				theta = 1-r;
			}
			else
			{
				theta = 1;
			}

			float xx = theta *newx;
			float yy = theta *newy;
			int sourcex = halfWidth + xx;
			int sourcey = halfHeight + yy;

			AI[i][k] = image[sourcey*10+int(sourcex/8)] << (sourcex%8) & 0x80;
		}
	}
}

/*

void distortion::correctionmatrix()
{

	float theta = 0;
	int newx,newy;

	int imageWidth = 80;
	int imageHeight = 60;

	int halfWidth = imageWidth / 2;
	int halfHeight = imageHeight / 2;

	for(int i = 0; i<60; i++)
	{

		for(int k =0 ; k<80; k++)
		{

			newx = k - halfWidth;
			newy = i - halfHeight;
			float distance = libutil::Math::Sqrt2(newx * newx + newy * newy);
			float r = float(distance /correctionRadius);
			if(r<1)
			{
				theta = 1-r;
			}
			else
			{
				theta = 1;
			}

			float xx = theta *newx;
			float yy = theta *newy;
			int sourcex = halfWidth + xx;
			int sourcey = halfHeight + yy;

			DI[i][k] = sourcey*100+sourcex;
		}
	}

}
*/
