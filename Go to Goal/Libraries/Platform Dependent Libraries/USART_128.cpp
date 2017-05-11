#include "USART_128.h"



void USART_Init( unsigned int baud, uint8_t n )
{
	sei();
	if(n)
	{
		//Initialize baud Rate(4800)
		UBRR1H=(unsigned char)(baud>>8);
		UBRR1L=(unsigned char)baud;
		//TX RX Enable
		UCSR1B|=(1<<RXEN0)|(1<<TXEN0);
		//Set 8-bit data, Parity disabled
		UCSR1C |=(3<<UCSZ10);
		
	}else
	{
		//Initialize baud Rate(9600)
		UBRR0H=(unsigned char)(baud>>8);
		UBRR0L=(unsigned char)baud;
		//TX RX Enable
		UCSR0B|=(1<<RXEN0)|(1<<TXEN0);//|(1<<UCSZ02);
		//Set 8-bit data, Parity disabled
		UCSR0C |= (3<<UCSZ00);
	}
	
}

void USART_InterruptEnable(uint8_t n)
{
	if(n)
	{
		UCSR1B|=(1<<RXCIE1);
	}
	else UCSR0B|=(1<<RXCIE0);
}

void USART_Transmitchar( unsigned char data, uint8_t n )
{
	if(n)
	{
		while ( !( UCSR1A & (1<<UDRE1)) );
		UDR1=data;
	}else
	{
		while ( !( UCSR0A & (1<<UDRE0)) );
		UDR0=data;
	}
}

unsigned char USART_Receive(uint8_t n )
{
	if(n)
	{
		while (! (UCSR1A & (1 << RXC1)) );
    	return UDR1;
	}else 
	{
		while (! (UCSR0A & (1 << RXC0)) );
    	return UDR0;
	}
}



void USART_TransmitString(char *str, uint8_t n)
{
	 while(*str>0)
	 {
		 USART_Transmitchar(*str,n);
		 //_delay_ms(1);
		 str++;
	 }
}

void USART_TransmitNumber(long int num, uint8_t n)
{
	if(num<0)
	{
		USART_Transmitchar('-',n);
		num=(-1)*num;
	}
	
	if(num >= 10){
    USART_TransmitNumber(num/10,n);
    num = num%10;
  }
  USART_Transmitchar(num+'0',n); /* n is between 0 and 9 */
	
}
/*
void USART0_TransmitBinary(int num)
{
	int i=0,j=0;
	while(num)
	{
		USART0_TransmitNumber(num%2);
		i++;
		num=num/2;
	}
	if(i!=8)
	{
		for(j=0;j<(8-i);j++)
		USART0_TransmitNumber(0);
	}
	
}*/
