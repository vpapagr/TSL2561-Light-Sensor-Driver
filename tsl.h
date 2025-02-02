/*
 * tsl.h
 *
 *  Created on: Jan 19, 2025
 *      Author: vpapagrigoriou
 */

#ifndef INC_TSL_H_
#define INC_TSL_H_

#include <main.h>
#include <math.h>

// I2C slave address for TSL2561
#define SLAVE_ADR 0x39 // 0b0111001

// Register map definitions
#define CONTROL 0x00          // Control register
#define TIMING 0x01           // Timing register
#define THRESHLOWLOW 0x02     // Low threshold lower byte
#define THRESHLOWHIGH 0x03    // Low threshold upper byte
#define THRESHHIGHLOW 0x04    // High threshold lower byte
#define THRESHHIGHHIGH 0x05   // High threshold upper byte
#define INTERRUPT 0x06        // Interrupt control register
#define CRC_REG 0x08          // CRC register
#define ID 0x0A               // ID register
#define DATA0LOW 0x0C         // ADC channel 0 lower byte
#define DATA0HIGH 0x0D        // ADC channel 0 upper byte
#define DATA1LOW 0x0E         // ADC channel 1 lower byte
#define DATA1HIGH 0x0F        // ADC channel 1 upper byte

// Command definitions
#define TSL_CMD 0x80          // Command bit
#define TSL_CLEAR 0x40        // Clear interrupt bit
#define TSL_WORD 0x20         // Word protocol bit
#define TSL_BLOCK 0x10        // Block protocol bit
#define TSL_ADDRESS 0x0F      // Address mask

// Function prototypes
void tsl_write(uint8_t reg, uint8_t data);
void tsl_read(uint8_t reg, uint8_t *receive_data);
void tsl_power_on();
void tsl_power_off();
int set_gain(int gain);
void enable_manual_operation();
void start_integration();
void stop_integration();
void get_id(uint8_t *part_no, uint8_t *rev_no);
void read_adc_channels(uint16_t *ch_0, uint16_t *ch_1);
float calculate_lux();

#endif /* INC_TSL_H_ */
