/*
 * Compass_Sensor.c
 *
 * Created: 25-12-2014 18:58:50
 *  Author: dell
 */ 
#include "Compass_Sensor.h"

#define F_SCL 100000UL // SCL frequency
#define Prescaler 1
#define TWBR_val ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2)

int16_t raw_x = 0;
int16_t raw_y = 0;
int16_t raw_z = 0;
uint16_t headingDegrees = 0;

void I2C_init(void){
	TWBR = TWBR_val;
}

uint8_t I2C_start(uint8_t address){
	// reset TWI control register
	TWCR = 0;
	// transmit START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	
	// check if the start condition was successfully transmitted
if((TWSR & 0xF8) != TW_START){ return 1; }

// load slave address into data register
TWDR = address;
// start transmission of address
TWCR = (1<<TWINT) | (1<<TWEN);
// wait for end of transmission
while( !(TWCR & (1<<TWINT)) );

// check if the device has acknowledged the READ / WRITE mode
uint8_t twst = TW_STATUS & 0xF8;
if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;

return 0;
}

uint8_t I2C_write(uint8_t data){
	// load data into data register
	TWDR = data;
	// start transmission of data
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	
	if( (TWSR & 0xF8) != TW_MT_DATA_ACK ){ return 1; }
	
	return 0;
}

uint8_t I2C_read_ack(void){
	
	// start TWI module and acknowledge data after reception
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA); 
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	// return received data from TWDR
	return TWDR;
}

uint8_t I2C_read_nack(void){
	
	// start receiving without acknowledging reception
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	// return received data from TWDR
	return TWDR;
}

void I2C_stop(void){
	// transmit STOP condition
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}

void init_HMC5883L(void){

	I2C_init();
	I2C_start(HMC5883L_WRITE);
	I2C_write(0x00); // set pointer to CRA
	I2C_write(0x70); // write 0x70 to CRA
	I2C_stop();

	I2C_start(HMC5883L_WRITE);
	I2C_write(0x01); // set pointer to CRB
	I2C_write(0xA0);
	I2C_stop();

	I2C_start(HMC5883L_WRITE);
	I2C_write(0x02); // set pointer to measurement mode
	I2C_write(0x00); // continous measurement
	I2C_stop();
}

uint16_t getHeading(void){

	I2C_start(HMC5883L_WRITE);
	I2C_write(0x03); // set pointer to X axis MSB
	I2C_stop();

	I2C_start(HMC5883L_READ);

	raw_x = ((uint8_t)I2C_read_ack())<<8;
	raw_x |= I2C_read_ack();

	raw_z = ((uint8_t)I2C_read_ack())<<8;
	raw_z |= I2C_read_ack();

	raw_y = ((uint8_t)I2C_read_ack())<<8;
	raw_y |= I2C_read_nack();

	I2C_stop();

	headingDegrees = (uint16_t)(atan2((double)raw_y,(double)raw_x) * 180 / 3.141592654 + 180);

	return headingDegrees;
}