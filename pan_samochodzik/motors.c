/*
 * motors.c
 *
 *  Created on: 09.11.2016
 *      Author: dawid
 */

#include "motors.h"


////////////////////////////////////////////////////////////
void init_motors(){
	SET_L_FOV;
	SET_L_BACK;
	SET_R_FOV;
	SET_R_BACK;
	SET_STOP_OFF;
	SET_L_DIR_ON;
	SET_R_DIR_ON;

}


void config_weel (void){

     //TCCR2	|= (1 << WGM21)
	//		| (1 << CS21) ;

     //OCR2=19;
    // OCR2 = 19*5;
     //TIMSK |= (1 << OCIE2);

     DDRB   |= WEEL;          //*******


     TIMSK |= (1<<OCIE1A) | (1 << TOIE1);// enable timer1 interrupts
     OCR1A = 1555;

     TCCR1B |= (1 << CS11); // prescaller
//			| ( 1 << WGM13)
//			| (1 << WGM12);

}

