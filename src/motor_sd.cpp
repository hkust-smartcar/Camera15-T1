/*
 * motor_sc.cpp

 *
 *  Created on: 2015�~4��15��
 *      Author: Benlai
 */
#include "motor_sd.h"




int SD::turn_left_encoder(int real_encoder , int servo, int servo_mid,int amplitute)
{
	int servo_difference = servo - servo_mid;

	int degree = angle(servo_difference, amplitute);

	if(mode == 0)
	{

		int rightencoder = radius_mode(real_encoder,'L' , degree);
		return rightencoder;

	}
	else if(mode == 1)
	{
		int rightencoder = angle_mode(real_encoder,'L' , degree);
		return rightencoder;

	}
	return -1;

}


int SD::turn_right_encoder(int real_encoder , int servo,int servo_mid,int amplitute)
{
	int servo_difference = servo - servo_mid;

	int degree = angle(servo_difference, amplitute);

	if(mode == 0)
	{

		int rightencoder = radius_mode(real_encoder,'R' , degree);
		return rightencoder;

	}
	else if(mode == 1)
	{
		int rightencoder = angle_mode(real_encoder,'R' , degree);
		return rightencoder;

	}
	return -1;

}



int SD::angle_mode(int encoder, char side , int degree)
{

	if(side == 'L')
	{
		int degreeL = degree + toe_in;

		int encoderL = radius_mode(encoder,side , degreeL);

		return encoderL;

	}
	else if(side == 'R')
	{
		int degreeR = degree - toe_in;

		int encoderR = radius_mode(encoder,side , degreeR);

		return encoderR;


	}
	else
	{

         return -1;

	}

	return -1;


}


int SD::radius_mode(int encoder, char side , int degree)
{
	libutil::Math tan;

	int de;
	if(side == 'L')
	{
		int leftencoder = 0;
		if(degree == 0)
		{
			left_encoder = encoder;
			return encoder;

		}
		else if(degree < 0)
		{

			de = -degree;
		}
		else
		{
			de = degree;
		}
		float radius = 1350/(de);


		if((degree * direction) < 0)
		{

			leftencoder = (((tan.ArcTan((((radius-(car_width/2))/(float)radius))) * 4)/(3.14159265359)) * encoder);
			left_encoder = leftencoder;

			return left_encoder;
		}
		else
		{

			leftencoder = ((tan.ArcTan((((radius+(car_width/2))/(float)radius))) * 4)/(3.14159265359) * encoder);
			left_encoder = leftencoder;

			return left_encoder;
		}

	}
	else if(side == 'R')
	{
		int rightencoder = 0;
		if(degree == 0)
		{

			right_encoder = encoder;
			return encoder;
		}
		else if(degree < 0)
		{

			de = -degree;
		}
		else
		{
			de = degree;
		}
		float radius = 1350/(de);

		if((degree * direction) < 0)
		{

			rightencoder = (((tan.ArcTan((((radius+(car_width/2))/(float)radius))) * 4)/(3.14159265359)) * encoder);
			right_encoder = rightencoder;
			return right_encoder;
		}
		else
		{

			rightencoder = ((tan.ArcTan((((radius-(car_width/2))/(float)radius))) * 4)/(3.14159265359) * encoder);
			right_encoder = rightencoder;
			return right_encoder;
		}

	}
	else
	{
		//printf("error side!!!!");
		return -1;
	}

	return -1;

}



int SD::angle(int servo_difference, int amplitute)
{
	int angle = (servo_difference / (float)amplitute) * maxium_turning_angle;
	return angle;

}








