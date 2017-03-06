
 #define F_CPU 8000000UL
#include <avr/io.h>
#include "avr/sfr_defs.h"
#include "USART_128.h"
#include <avr/interrupt.h>
#include <util/delay.h>

#define PWM_FL		OCR1A
#define PWM_BL		OCR1B
#define PWM_CL		OCR1C
#define PWM_FR		OCR3A
#define PWM_BR		OCR3B
#define PWM_CR		OCR3C

#define DIR_DDR     DDRC
#define DIR_PORT     PORTC
#define DIR_FL		PINC0
#define DIR_BL		PINC1
#define DIR_CL		PINC2
#define DIR_FR		PINC3
#define DIR_BR		PINC4
#define DIR_CR		PINC5

void forward(void);
void backward(void);
void sharp_left(void);
void sharp_right(void);
void left_Turn(void);
void right_Turn(void);
void stop(void);

char m;


int main(void)
{	DDRA=0XFF;
	DDRE=0xFF;
	DDRB=0xFF;DDRC=0xFF;
	DIR_DDR = 0xFF; //direction pin
	
	sei();

	USART_Init(51,1);				//9600 baud
	USART_InterruptEnable(1);
	USART_Init(51,1);
	USART_InterruptEnable(1);
	
	TCCR1A |= (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1)|(1<<COM1C1);   //Fast PWM,Non Inverting mode
	TCCR1B |=(1<<WGM12)|( 1<<WGM13)|(1<<CS11);

	TCCR3A |=( 1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);//Fast PWM ,Non Inverting mode
	TCCR3B |=(1<<WGM32)|( 1<<WGM33)|(1<<CS31);

	ICR1 =10000;
	ICR3 =10000;

	while (1)
	{
	}

}
ISR(USART1_RX_vect)
{
	m = USART_Receive(1);
	//USART_Transmitchar(m,0);
	switch (m)
	{
		case 'F':
			forward();
			break;

		case 'B':
			backward();
			break;

		case 'R':
			sharp_right();		
			break;

		case 'L':
			sharp_left();
			break;

		case 'q' :
			PWM_BL = 10000;
			PWM_CL = 10000;
			PWM_FL = 10000;
			PWM_BR = 10000;
			PWM_CL = 10000;
			PWM_CR = 10000;
			break;

		default:
			stop();
		
	}

	if (m>='0' && m<='9')
	{
		PWM_BL = 10000*((m-'0')/10.0);
		PWM_CL = 10000*((m-'0')/10.0);
		PWM_FL = 10000*((m-'0')/10.0);
		PWM_BR = 10000*((m-'0')/10.0);
		PWM_CR = 10000*((m-'0')/10.0);
		PWM_FR = 10000*((m-'0')/10.0);
	}

	USART_TransmitNumber(PWM_BL,1);
	USART_Transmitchar(0x0D,1);
	USART_TransmitNumber(DIR_PORT,1);
	USART_Transmitchar(0x0D,1);	
}


void forward(void)
{
	DIR_PORT= 0b00000000;
	USART_TransmitString("FORWARD",1);
}

void backward(void)
{
	DIR_PORT = 0b11111111;
	USART_TransmitString("BACKWARD",1);
}


void sharp_right(void)
{	
	DIR_PORT =  (1<<DIR_CR)|(1<<DIR_BR)|(1<<DIR_FR);
	USART_TransmitString("SHRAP RIGHT",1);
}


void sharp_left(void)
{
	DIR_PORT = (1<<DIR_CL) | (1<<DIR_BL) |(1<<DIR_FL);
	USART_TransmitString("SHARP LEFT",1);
}


void stop(void)
{
	PWM_BL = 0;
	PWM_CL = 0;
	PWM_FL = 0;
	PWM_BR = 0;
	PWM_CL = 0;
	PWM_CR = 0;
}



