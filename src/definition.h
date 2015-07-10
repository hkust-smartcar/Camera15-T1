//image info
#define HEIGHT 58
#define WIDTH 78
#define CS 0
#define CE 58
#define RS 0	//row start
#define RE 78	//row end

//status
#define INIT_STATE 	 0
#define STRAIGHT 	 1
#define CROSSROAD 	 2
#define RIGHT_ANGLE  3
#define TURNING 	 4
#define OUT_OF_BOUND 5
#define BLACK_GUIDE	 6

#define MIDPOINT_REF 42 //37

//data info
#define WHITECOUNT 0
#define ISWHITE 1
#define ISBLACK 2
#define WHITEAT 3

//checking midpoint
#define MS 25	//check start
#define ME 35   //check end

//for array use
#define LEFT 0
#define RIGHT 1

#define TOP 0
#define BOTTOM 1

#define ROW_VALUE 0
#define SAME_COUNT 1
#define START_AT 2

#define START 0
#define END 1

//servo
#define SERVO_ERR 1350
#define SERVO_MID_DEGREE 8950  //9500
#define SERVO_AMPLITUDE  5000 //3700
