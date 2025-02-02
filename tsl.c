/*
 * tsl.c
 *
 *  Created on: Jan 19, 2025
 *      Author: vpapagrigoriou
 */

#include <tsl.h>

/**
 * @brief Writes data to a TSL2561 register via I2C.
 *
 * This function uses the HAL library to send data to
 * the TSL2561 register. It first sends the register address
 * and then the data to be written.
 *
 * @param reg The register address where data will be written.
 * @param data The data to be written to the register.
 */
void tsl_write(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c1, SLAVE_ADR << 1, reg, 1, &data, 1, 1000);
}

/**
 * @brief Reads data from a TSL2561 register via I2C.
 *
 * This function uses the HAL library to read data from
 * the TSL2561 register and store it in the provided variable.
 *
 * @param reg The register address from which data will be read.
 * @param receive_data Pointer to the variable where the read data will be stored.
 */
void tsl_read(uint8_t reg, uint8_t *receive_data)
{
    HAL_I2C_Mem_Read(&hi2c1, SLAVE_ADR << 1, reg, 1, receive_data, 1, 1000);
}

/**
 * @brief Powers on the TSL2561 sensor.
 *
 * This function reads the control register, modifies the first two bits,
 * and writes back to the register to power on the sensor.
 */
void tsl_power_on()
{
    uint8_t receive_data;

    // Read the control register
    tsl_read(TSL_CMD | CONTROL, &receive_data);

    // Power on the sensor
    receive_data |= (1 << 0);
    receive_data |= (1 << 1);

    // Write the new data to the control register
    tsl_write(TSL_CMD | CONTROL, receive_data);
}

/**
 * @brief Powers off the TSL2561 sensor.
 *
 * This function reads the control register, clears the power bits,
 * and writes back to the register to power off the sensor.
 */
void tsl_power_off()
{
    uint8_t power_off_data;

    // Read the control register
    tsl_read(TSL_CMD | CONTROL, &power_off_data);

    // Power off the sensor
    power_off_data &= ~(1 << 1);
    power_off_data &= ~(1 << 0);

    // Write the new data to the control register
    tsl_write(TSL_CMD | CONTROL, power_off_data);
}

/**
 * @brief Sets the gain of the sensor.
 *
 * This function reads the timing register and modifies the gain bit.
 * The gain can be set to 0 or 1.
 *
 * @param gain Gain value (0 or 1).
 * @return 0 on success, -1 on failure.
 */
int set_gain(int gain)
{
    uint8_t timing_value;

    tsl_read(TSL_CMD | TIMING, &timing_value);

    if (gain == 0) {
        // Clear bit at position 4
        timing_value &= ~(1 << 4);
        tsl_write(TSL_CMD | TIMING, timing_value);
        return 0;
    } else if (gain == 1) {
        // Set bit at position 4
        timing_value |= (1 << 4);
        tsl_write(TSL_CMD | TIMING, timing_value);
        return 0;
    } else {
        return -1;
    }
}

/**
 * @brief Enables manual operation mode of the TSL2561.
 *
 * This function modifies the timing register to enable manual operation mode.
 */
void enable_manual_operation()
{
    uint8_t integ;

    tsl_read(TSL_CMD | TIMING, &integ);

    // Enable manual operation (bit 0)
    integ |= (1 << 0);

    tsl_write(TSL_CMD | TIMING, integ);
}

/**
 * @brief Starts the light measurement integration process.
 *
 * This function modifies the timing register to start the
 * integration process for light measurement.
 */
void start_integration()
{
    uint8_t manual;

    tsl_read(TSL_CMD | TIMING, &manual);

    // Start integration (bit 3)
    manual |= (1 << 3);

    tsl_write(TSL_CMD | TIMING, manual);
}

/**
 * @brief Stops the light measurement integration process.
 *
 * This function modifies the timing register to stop the
 * integration process for light measurement.
 */
void stop_integration()
{
    uint8_t manual;

    tsl_read(TSL_CMD | TIMING, &manual);

    // Stop integration (bit 3)
    manual &= ~(1 << 3);

    tsl_write(TSL_CMD | TIMING, manual);
}

/**
 * @brief Reads the ID of the TSL2561.
 *
 * This function reads the sensor's ID and returns the part
 * number and revision number in separate variables.
 *
 * @param part_no Pointer to store the part number.
 * @param rev_no Pointer to store the revision number.
 */
void get_id(uint8_t *part_no, uint8_t *rev_no)
{
    uint8_t reg_id;

    tsl_read(TSL_CMD | ID, &reg_id);

    // Read and store the part number and revision
    *rev_no = reg_id & 0x0f;
    *part_no = (reg_id >> 4) & 0x0f;
}

/**
 * @brief Reads ADC channel data.
 *
 * This function reads the ADC data from both channels
 * and stores the values in the provided pointers.
 *
 * @param ch_0 Pointer to store channel 0 data.
 * @param ch_1 Pointer to store channel 1 data.
 */
void read_adc_channels(uint16_t *ch_0, uint16_t *ch_1)
{
    uint8_t data0low, data0high, data1low, data1high;

    tsl_read(TSL_CMD | DATA0LOW, &data0low);
    tsl_read(TSL_CMD | DATA0HIGH, &data0high);
    *ch_0 = (data0high << 8) | data0low;

    tsl_read(TSL_CMD | DATA1LOW, &data1low);
    tsl_read(TSL_CMD | DATA1HIGH, &data1high);
    *ch_1 = (data1high << 8) | data1low;
}

/**
 * @brief Calculates the lux value from ADC channel data.
 *
 * This function reads ADC data from the sensor, calculates
 * the ratio, and computes the lux value based on predefined formulas.
 *
 * @return Calculated lux value.
 */
float calculate_lux()
{
    float coefficients, lux;
    uint16_t ch_0, ch_1;

    read_adc_channels(&ch_0, &ch_1);

    if (ch_0 == 0) {
        return 0;  // Avoid division by zero
    }

    coefficients = ((float)ch_1) / ((float)ch_0);

    if (coefficients > 0 && coefficients <= 0.50) {
        lux = 0.0304 * ch_0 - 0.062 * ch_0 * pow(coefficients, 1.4);
    } else if (coefficients > 0.50 && coefficients <= 0.61) {
        lux = 0.0224 * ch_0 - 0.031 * ch_1;
    } else if (coefficients > 0.61 && coefficients <= 0.80) {
        lux = 0.0128 * ch_0 - 0.0153 * ch_1;
    } else if (coefficients > 0.80 && coefficients <= 1.30) {
        lux = 0.00146 * ch_0 - 0.0012 * ch_1;
    } else if (coefficients > 1.30) {
        lux = 0;
    }

    return lux;
}
