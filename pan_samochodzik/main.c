/*
 * main.c
 *
 *  Created on: 05.11.2016
 *      Author: dawid
 */
/*
 * main.c
 *
 *  Created on: 05.11.2016
 *      Author: dawid
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "motors.h"

#include "twi.h"

#define SENSOR_1_PIN PC0
#define SENSOR_2_PIN PC1
#define SENSOR_3_PIN PC2
#define BATTERY_PIN PC3

#define SENSOR_1_TRIGGER PD3
#define SENSOR_2_TRIGGER PD4
#define SENSOR_3_TRIGGER PD6

#define SENSOR_1_TRIGGER_PORT PORTD
#define SENSOR_1_TRIGGER_DDR DDRD
#define SENSOR_2_TRIGGER_PORT PORTD
#define SENSOR_2_TRIGGER_DDR DDRD
#define SENSOR_3_TRIGGER_PORT PORTD
#define SENSOR_3_TRIGGER_DDR DDRD

#define SENSOR_1_TRIG_ON SENSOR_1_TRIGGER_PORT |= (1 << SENSOR_1_TRIGGER )
#define SENSOR_1_TRIG_OFF SENSOR_1_TRIGGER_PORT &=~ (1 << SENSOR_1_TRIGGER )
#define SENSOR_2_TRIG_ON SENSOR_2_TRIGGER_PORT |= (1 << SENSOR_2_TRIGGER )
#define SENSOR_2_TRIG_OFF SENSOR_2_TRIGGER_PORT &=~ (1 << SENSOR_2_TRIGGER )
#define SENSOR_3_TRIG_ON SENSOR_3_TRIGGER_PORT |= (1 << SENSOR_3_TRIGGER )
#define SENSOR_3_TRIG_OFF SENSOR_3_TRIGGER_PORT &=~ (1 << SENSOR_3_TRIGGER )
#define THRESHOLD 4

#define SENSOR_CHANGE_TIME 30
#define DELAY_BETWEEN_MEASURE 3000

#define ECHO_1_TRIGGER PD2
#define ECHO_1_PIN PD0

#define ECHO_1_DDR DDRD
#define ECHO_1_PORT PORTD
#define ECHO_1_PINPORT PIND
#define ECHO_1_TRIGGER_ON ECHO_1_PORT |= (1<< ECHO_1_TRIGGER)
#define ECHO_1_TRIGGER_OFF ECHO_1_PORT &=~ (1<< ECHO_1_TRIGGER)
#define ECHO_1_PIN_IS_ON PIND & (1<<ECHO_1_PIN)
#define ECHO_THRESHOLD 100

volatile unsigned char timer_counter;

volatile uint16_t us10=0;

unsigned char diode_off_value;
unsigned char diode_on_value;
volatile uint16_t echo_1_distance;
unsigned char messageBuf[TWI_BUFFER_SIZE];
unsigned char TWI_slaveAddress;

unsigned char get_value(unsigned char adc_number);

unsigned char sensor_buf;
volatile uint32_t echo_buf;
volatile uint8_t echo_val;
volatile uint8_t i2ccomm=0;

uint8_t main_loop_counter;
int main(){


	sei();
	//TWI SETUP AS A SLAVE


	TWI_slaveAddress = 0x44;
	TWI_Slave_Initialise( (unsigned char)((TWI_slaveAddress<<TWI_ADR_BITS) | (TRUE<<TWI_GEN_BIT) ));

	TWI_Start_Transceiver();


	//TIMER SETUP
	timer_counter = 0;


	//ADC CONFIGURATION

    ADMUX |= (1<<REFS0)
    		|(1<<ADLAR);

    ADCSRA|=(1<<ADEN)
    		|(1<<ADPS2)
    		|(1<<ADPS1)
			|(1<<ADPS0);

	config_weel();
	init_motors();

	SENSOR_1_TRIGGER_DDR |= (1 << SENSOR_1_TRIGGER);
	SENSOR_2_TRIGGER_DDR |= (1 << SENSOR_2_TRIGGER);
	SENSOR_3_TRIGGER_DDR |= (1 << SENSOR_3_TRIGGER);

	ECHO_1_DDR |= (1<<ECHO_1_TRIGGER);

	angle_val = 50;


	while(1){

		_delay_us(90);

		if (L_DIR_FLAG) {
			if (timer_counter == MIN_SIGNAL_VALUE) {
				L_BACK_OFF;
				L_FOV_ON;
			}
			if (timer_counter == l_motor_pwm) {
				L_FOV_OFF;
			}
		} else {
			if (timer_counter == MIN_SIGNAL_VALUE) {
				L_FOV_OFF;
				L_BACK_ON;
			}

			if (timer_counter == l_motor_pwm)
				L_BACK_OFF;
		}

		if (R_DIR_FLAG) {
			if (timer_counter == MIN_SIGNAL_VALUE) {
				R_BACK_OFF;
				R_FOV_ON;
			}
			if (timer_counter == r_motor_pwm)
				R_FOV_OFF;
		} else {
			if (timer_counter == MIN_SIGNAL_VALUE) {
				R_FOV_OFF;
				R_BACK_ON;
			}
			if (timer_counter == r_motor_pwm)
				R_BACK_OFF;
		}

		timer_counter++;
		if (ECHO_1_PIN_IS_ON)
			echo_1_distance++;
		else if (echo_1_distance) {

			echo_val = echo_1_distance ;

			echo_1_distance = 0;
		}

		if (timer_counter > MAX_SIGNAL_VALUE) {
			timer_counter = MIN_SIGNAL_VALUE;
			main_loop_counter++;
		}

		if ((main_loop_counter > 100)) {
			main_loop_counter = 0;

			diode_off_value = get_value(SENSOR_1_PIN);

			SENSOR_1_TRIG_ON;
			_delay_us(SENSOR_CHANGE_TIME);
			diode_on_value = get_value(SENSOR_1_PIN);
			SENSOR_1_TRIG_OFF;
			if (diode_off_value > diode_on_value)
				if (diode_off_value - diode_on_value > THRESHOLD)
					;	//sensor_buf=1;
			_delay_us(DELAY_BETWEEN_MEASURE);
			diode_off_value = get_value(SENSOR_2_PIN);

			SENSOR_2_TRIG_ON;
			_delay_us(SENSOR_CHANGE_TIME);
			diode_on_value = get_value(SENSOR_2_PIN);
			SENSOR_2_TRIG_OFF;
			if (diode_off_value > diode_on_value)
				if (diode_off_value - diode_on_value > THRESHOLD)
					;	//sensor_buf=1;
			_delay_us(DELAY_BETWEEN_MEASURE);
			diode_off_value = get_value(SENSOR_3_PIN);

			SENSOR_3_TRIG_ON;
			_delay_us(SENSOR_CHANGE_TIME);
			diode_on_value = get_value(SENSOR_3_PIN);
			SENSOR_3_TRIG_OFF;
			if (diode_off_value > diode_on_value)
				if (diode_off_value - diode_on_value > THRESHOLD)
					;	//sensor_buf=1;

			OCR1A = 1950 + ( ( angle_val * 195 ) / 10 );//update weel angle

			ECHO_1_TRIGGER_ON;
			_delay_us(10);
			ECHO_1_TRIGGER_OFF;

		}



		}//end of infinite loop*/
		//return 0;
	}


////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////

ISR(TIMER1_COMPA_vect){

	OFF_WEEL;

	TCNT1 = (255*256 - 39015) + OCR1A;
	//TCNT1H = OCR1AH;
	//TCNT1L = OCR1AL +1;
}

ISR(TIMER1_OVF_vect){
	ON_WEEL;
}

//////////////////////////////////////////////////////////////////////////////////

	unsigned char get_value(unsigned char adc_number){

		ADMUX &=~ 0b0000111;
		ADMUX |= adc_number;//choose ADC channel

		ADCSRA |= (1<<ADSC);//Start measure

		while(ADCSRA&(1<<ADSC));//Wait until measure finish

		return ADCH;

	}

