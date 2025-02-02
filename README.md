# TSL2561-Light-Sensor-Driver
This repository contains a C driver for the TSL2561 light-to-digital converter. The driver provides functions to communicate with the TSL2561 sensor via I2C, allowing for the measurement of ambient light in lux.

## Features

- I2C communication with TSL2561 sensor
- Power management functions
- Gain control for light sensitivity adjustment
- Manual and automatic integration control
- ADC channel reading
- Lux calculation based on sensor data

## Files 

- **tsl.c**: Implementation of functions for controlling and reading data from the TSL2561 sensor.
- **tsl.h**: Header file containing function prototypes and register definitions.

## Getting Started

### Prerequisites
- STM32 HAL Library (or compatible microcontroller HAL)
- I2C peripheral enabled on the microcontroller
- Math library for pow() function

### Hardware Connections
- SCL: I2C clock line
- SDA: I2C data line
- VDD: 2.7V to 3.6V power supply
- GND: Ground

## Usage

1. **Initialize I2C**: Ensure your microcontroller's I2C is initialized.
2. **Power On the Sensor**:
   ```
   tsl_power_on();
   ```
3. **Set Gain (optional)**:
   ```
   set_gain(0); // Low gain
   ```
4. **Read Light Data**:
   ```
   float lux = calculate_lux();
   ```
5. **Power Off the Sensor (optional)**:
   ```
   tsl_power_off();
   ```
## Example
```
int main(void)
{
  float lux;

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();          
  MX_USART2_UART_Init();      
  MX_I2C1_Init();             

  tsl_power_on();
  uint8_t part_no;  
  uint8_t rev_no;  
  get_id(&part_no, &rev_no);  

  while (1)
  {
	   lux = calculate_lux();
	   printf("lux=%f\n",lux);
  }
}
```
## Contributing
Contributions are welcome! Feel free to submit issues and pull requests to improve the driver.

## Author
V. Papagrigoriou

For more information on the TSL2561 sensor, refer to the TSL2561 datasheet.
