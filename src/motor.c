/*************************************************
Firmware developed for AX12 servo
ATMega8 processor at 16MHz
by: Ricardo Marinheiro
February 2011
robosavvy
Move Interactive
*************************************************/


#include "motor.h"

//extern uint16_t  PWM=1000;
extern uint8_t PRESENT_LOAD_LOW;
extern uint8_t PRESENT_LOAD_HIGH;
extern uint8_t TORQUE_ENABLE;

int8_t SetPWM(uint16_t pwm)
{
	uint8_t sreg;
	if(pwm!= PRESENT_LOAD_LOW + (PRESENT_LOAD_HIGH<<8) /*PWM*/)
	{
		
		Motor_Port &= ~(Motor_Clockwise|Motor_Anticlockwise);
		/* Save Global Interrupt Flag */
		sreg = SREG;
		/* Disable interrupts */
		cli();
		//TCCR1A =(bit(WGM10)|bit(WGM11));
		TCCR1A=0;
		if(pwm>1000)
		{
			OCR1A = (pwm-1000);
			TCCR1A |= bit(COM1A1);
			Motor_Port |= Motor_Anticlockwise;
		}
		else
		if(pwm<1000)
		{
			OCR1B = (1000-pwm);
			TCCR1A |= bit(COM1B1);
			Motor_Port |= Motor_Clockwise;
		}

		sei();
		SREG = sreg;
		PRESENT_LOAD_LOW=pwm & 0x00FF;
		PRESENT_LOAD_HIGH= pwm>>8;
		
		//		PWM=pwm;
	}
	return 1;

}





void InitMotor(void)
{
	//Init Timer 1 for PWM

	Motor_Port&= ~(Motor_Clockwise|Motor_Anticlockwise);	

	TCNT1 = 0;
	TCCR1A = 0;
	TCCR1B = 0;

	ICR1=1000; 
	
	TCCR1B = (bit(WGM13)|/*bit(CS10)*/bit(CS11)); // CS10=10KHz PWM   CS11 = 1.2 KHz

	//TCCR1A = (bit(WGM10)|bit(WGM11));
	//TCCR1B = (bit(WGM12)|/*bit(CS10)*/bit(CS11)); // CS10=10KHz PWM   CS11 = 1.2 KHz
	// Set the PWM duty cycle to zero.
	OCR1A = 0;
	OCR1B = 0;
	SetPWM(1000);
}

