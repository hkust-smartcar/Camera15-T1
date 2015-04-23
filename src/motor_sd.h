/*
 * motor_sc.h
 *
 *  Created on: 2015/4/15
 *      Author: Benlai
 */

#ifndef SRC_MOTOR_SD_H_
#define SRC_MOTOR_SD_H_


#include <libutil/math.h> //use Arctan


/*Use to choose which mode to use, Radius_Mode suits for all kind of car, and Angle_Mode need
to specific some kind of parameters like the maximum angle can turn and the toe_in angle.*/
#define Radius_Mode 0
#define Angle_Mode 1
#define Road_Mode 2



#define mode Radius_Mode

//For all mode, used to calculate something
#define car_width 14
#define maxium_turning_angle 60 //the angle is respect to the length of the car


//Just for Angle_Mode only for precise calculation
#define toe_in 4   //units in degree(around -8 to 8), minus mean toe out


/*if maximum turning left angle and turning right angle is not equal then I think the
mechanism of the car is bull-shit*/


//Just for Road_Mode only for very very precise calculation
#define road_length 45
#define road_maximem_radius 30

class SD
{
public:
	//Do all the things below
	//The first parameter(encoder) is just the speed you want to set;
	//The second parameter(servo) is now the servo angle, you can GetDegree() to get back the servo value
	//The third parameter(servo_mid) is the servo value that the wheels are moving front
	//The forth parameter(amplitute) is the amplitute for the maximum value servo can turn

	//software differential based on servo degree and the encoder value
	//It returns the real left encoder value based on your encoder input
	int turn_left_encoder(int encoder , int servo, int servo_mid,int amplitute);

	//software differential based on servo degree and the encoder value
	//It returns the real right encoder value based on your encoder input
	int turn_right_encoder(int encoder , int servo, int servo_mid,int amplitute);

	int angle(int servo_difference, int amplitute);

	int angle_mode(int encoder, char side , int degree);

	int radius_mode(int encoder, char side, int degree);

private:

	//can store up to 3 recent data, [0] is the most recent data;
	int left_encoder;
	int right_encoder;


};


#endif /* SRC_MOTOR_SC_H_ */
