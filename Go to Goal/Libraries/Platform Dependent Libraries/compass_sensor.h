#ifndef COMPASS_SENSOR
#define COMPASS_SENSOR

#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sfr_defs.h>
#include <math.h>
#include <util/twi.h>
#include <stdlib.h>

#define I2C_READ 0x01
#define I2C_WRITE 0x00
#define HMC5883L_WRITE 0x3C
#define HMC5883L_READ 0x3D



void I2C_init(void);
uint8_t I2C_start(uint8_t address);
uint8_t I2C_write(uint8_t data);
uint8_t I2C_read_ack(void);
uint8_t I2C_read_nack(void);
void I2C_stop(void);
void init_HMC5883L(void);
uint16_t getHeading(void);

#endif 