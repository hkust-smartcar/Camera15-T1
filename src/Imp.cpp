/*
 * Imp.cpp
 *
 *  Created on: 2015年6月12日
 *      Author: Benlai
 */

#include "Imp.h"




Imp::Imp() {}

Imp::~Imp() {}






int Imp::GetPixel(const Byte* src, const uint8_t x, const uint8_t y)
{
//	const int offset = x/8 + (y * image_width / 8);
	const int offset = x/8 + (y * 80 / 8);

//	return (src[offset] << (x%8) & 0x80) ? 0 : 1;
	return (src[offset] << (x%8) & 0x80) ? 1 : 0;

}




void Imp::medianFilter(const Byte* src, bool bitmap[58][78])
{

	for(int i = 1; i < 59; i++)
	{

		for(int w = 1 ; w < 79; w++ )
		{

				int white = 0;

				for(int z = 0 ; z<3; z++)
				{
					if(GetPixel(src,w-1+z,i-1) == 0)
					{
						white++;
					}
				}
				for(int z = 0 ; z<3; z++)
				{
					if(GetPixel(src,w-1+z,i) == 0)
					{
						white++;
					}
				}
				for(int z = 0 ; z<3; z++)
				{
					if(GetPixel(src,w-1+z,i+1) == 0)
					{
						white++;
					}
				}
				if(white > 4)
				{
					//				mf[(i-1)*78+w-1] = 0;
					bitmap[i-1][w-1] = false;
				}
				else
				{
					//				mf[(i-1)*78+w-1] = 1;
					bitmap[i-1][w-1] = true;
				}
			}


		}

}

void Imp::medianFilterPrint(const Byte* src, libsc::St7735r *lcd)
{


	for(int i = 1; i < 59; i++)
	{
		for(int w = 1 ; w < 79; w++ )
		{
			int white = 0;

			for(int z = 0 ; z<3; z++)
			{
				if(GetPixel(src,w-1+z,i-1) == 0)
				{
					white++;
				}
			}
			for(int z = 0 ; z<3; z++)
			{
				if(GetPixel(src,w-1+z,i) == 0)
				{
					white++;
				}
			}
			for(int z = 0 ; z<3; z++)
			{
				if(GetPixel(src,w-1+z,i+1) == 0)
				{
					white++;
				}
			}
			if(white > 4)
			{
				mf[(i-1)*78+w-1] = 0;
			}
			else
			{
				mf[(i-1)*78+w-1] = 1;
			}


		}

	}

	lcd->SetRegion({0, 70, 78, 58});
	lcd->FillBits(0, 0xFFFF, mf, 78 * 58);

}
