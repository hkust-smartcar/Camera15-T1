#include "VerticalImageProcess.h"

namespace camera{

VerticalImageProcess::VerticalImageProcess(){
	for(int8_t i=RS; i<RE; i++){
		margin[i][TOP] = CS;
		margin[i][BOTTOM] = CE-1;
	}

	for(int8_t i=0; i<WIDTH; i++){
		cont_top_margin[i][ROW_VALUE]=100;
		cont_bottom_margin[i][ROW_VALUE]=100;
	}
}

void VerticalImageProcess::Analyze(bool binary[HEIGHT][WIDTH]){

/* +-----------------------+
 * |           |		   |
 * |           v           |
 * |-----------------------|
 * |***********************|
 * |*****------------------|
 * |**---      ^           |
 * |--         |		   |
 * +-----------------------+
*/
	//initialize

	highest_cont_margin = CE-1;
	lowest_cont_margin = CS;

	top_cont_last = 0;
	bottom_cont_last = 0;


	for(uint8_t j=RS; j<RE; j++){

		//initialize

		for(int8_t i=0; i<WIDTH; i++){
			cont_top_margin[i][ROW_VALUE]=100;
			cont_bottom_margin[i][ROW_VALUE]=100;
		}

		bool prev1 = binary[CE-1][j];
		bool prev2 = binary[HEIGHT/6][j];

		margin[j][TOP] = CS;
		margin[j][BOTTOM] = CE-1;

		//if bottom is black -> assume that column is black
		if(binary[CE-1][j]){
			margin[j][BOTTOM] = CE-1;
		}

		//if bottom is white -> first black = margin
		else{
			//scan from bottom: until black
			for(int8_t i=CE-1; i>=HEIGHT/6; i--){
				if(binary[i][j]!=prev1 && !prev1){
					margin[j][BOTTOM] = i;
					break;
				}
				prev1 = binary[i][j];
			}
		}

		//scan from top: until black or bottom margin
		for(int8_t i=HEIGHT/6; i<CE; i++){
			if((binary[i][j]!=prev2 && !prev2)||i==margin[j][BOTTOM]){
				margin[j][TOP] = i;
				break;
			}
			prev2 = binary[i][j];
		}

	}

	//find continuous edges
	//cont_top_margin/cont_bottom_margin[#][0]=value of margin; [#][1]=count of same margin

	//cont_top_margin[3]=1		cont_top_margin[3]=25
	//   |						 |
	//   v						 v
	/* 0 +-----------------------+
	 * 1 |            		     |
	 * 2 |                       |
	 * 3 |-----------------------|
	 * 4 |***********************|
	 * 5 |*****------------------|
	 * 6 |**---                  |
	 * 7 |--          		     |
	 * 8 +-----------------------+
	     0123456789012345678901234*/
	/* 0 +-----------------------+
	 * 1 |            		     |
	 * 2 |                       |
	 * 3 |-----------------------|
	 * 4 |***********************|
	 * 5 |*****------------------|
	 * 6 |**---                  |
	 * 7 |--          		     |
	 * 8 +-----------------------+
	 *   ^     ^        ^
	 * 	 |     |        |
	 *[0][0]=7 [3][0]=5 [3][0]=5
	 *[0][1]=1 [3][1]=1 [3][1]=11
	 */

	/*only save when count of same margin > threshold*/

	uint8_t prev_top_margin = margin[RS][TOP];
	uint8_t prev_bottom_margin = margin[RS][BOTTOM];

	//counter
	uint8_t t_count = 1;
	uint8_t b_count = 1;
	uint8_t top_cont_count=0;
	uint8_t bottom_cont_count=0;

	for(int8_t j=RS+1; j<RE; j++){

		if(abs(margin[j][TOP]-prev_top_margin)>3){
			t_count=1;
			prev_top_margin = margin[j][TOP];
		}
		else{

			if(t_count==threshold){
				cont_top_margin[top_cont_count][START_AT] = j;
				cont_top_margin[top_cont_count][ROW_VALUE] = margin[j][TOP];
				cont_top_margin[top_cont_count][SAME_COUNT] = threshold;
				t_count++;
				top_cont_count++;
			}
			else if(t_count>threshold){
				t_count++;
				cont_top_margin[top_cont_count-1][SAME_COUNT]++;
			}
			else{
				t_count++;
			}
		}

		if(abs(margin[j][BOTTOM]-prev_bottom_margin)>3){
			b_count=1;
			prev_bottom_margin = margin[j][BOTTOM];
		}
		else{
			if(b_count==threshold){
				cont_bottom_margin[bottom_cont_count][START_AT] = j;
				cont_bottom_margin[bottom_cont_count][ROW_VALUE] = margin[j][BOTTOM];
				cont_bottom_margin[bottom_cont_count][SAME_COUNT] = threshold;
				b_count++;
				bottom_cont_count++;
			}
			else if(b_count>threshold){
				b_count++;
				cont_bottom_margin[bottom_cont_count-1][SAME_COUNT]++;
			}
			else{
				b_count++;
			}
		}

	}

	//calculate total length of continuous margin

	top_len = 0;
	bottom_len = 0;

	for(int8_t i=0; i<WIDTH; i++){

		if(cont_top_margin[i][ROW_VALUE]!=100 && cont_top_margin[i][ROW_VALUE]>HEIGHT/6){

				top_len+=cont_top_margin[i][SAME_COUNT];
				top_cont_last = i;

				if(cont_top_margin[i][ROW_VALUE]<highest_cont_margin){
					highest_cont_margin = cont_top_margin[i][ROW_VALUE];
				}

		}

		if(cont_bottom_margin[i][ROW_VALUE]!=100 && cont_top_margin[i][ROW_VALUE]<CE-1){

				bottom_len+=cont_bottom_margin[i][SAME_COUNT];
				bottom_cont_last = i;

				if(cont_bottom_margin[i][ROW_VALUE]>lowest_cont_margin){
					lowest_cont_margin = cont_bottom_margin[i][ROW_VALUE];
				}
		}
	}

}

}
