/**\mainpage
 * Copyright (C) 2022 SIP Lab ETH Zurich
 *
 * File         iim42652.c
 * Date         29.12.2022
 * Author       Manuel Meier
 * Version      1.0.0
 *
 */
#include "iim42652.h"
#include <string.h>
#include <stdio.h>

/*!
 * @brief This internal API is used to validate the device pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of iim42652_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t iim42652_null_ptr_check(const struct iim42652_dev *dev);

/*!
 * @brief This internal API decodes fifo data into arrays of sensor
 * values.
 *
 * @param[in] dev : Structure instance of iim42652_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static void decode_fifo_data(const uint8_t* fifo_data, const uint16_t fifo_size, uint32_t* gyro_data, uint8_t* gyro_count, uint32_t* accel_data, uint8_t* accel_count, const uint8_t arr_size);

/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t iim42652_null_ptr_check(const struct iim42652_dev *dev)
{
        int8_t rslt;

        if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->spi_dev == NULL)) {
                /* Device structure pointer is not valid */
                rslt = SENSOR_E_NULL_PTR;
        } else {
                /* Device structure is fine */
                rslt = SENSOR_OK;
        }
        return rslt;
}

static void decode_fifo_data(const uint8_t* fifo_data, const uint16_t fifo_size, uint32_t* gyro_data, uint8_t* gyro_count, uint32_t* accel_data, uint8_t* accel_count, const uint8_t arr_size){
        *gyro_count  = 0;
        *accel_count = 0;
        const uint8_t* fifo_ptr = fifo_data;
        while(fifo_ptr < fifo_data+fifo_size){
                uint8_t poffs = 0;
                if(fifo_ptr[0]&0x80){ // packet is empty, should not happen normally
                        fifo_ptr++;
                        continue;
                }
                if(fifo_ptr[0]&0x40){ // packet contains accelerometer data
                        accel_data[             *accel_count] = ( *((int16_t*) (fifo_ptr+1)) );    // x value if endian was not adjusted, this would be fifo_ptr[1]<<8|fifo_ptr[2];
                        accel_data[  arr_size + *accel_count] = ( *((int16_t*) (fifo_ptr+3)) );    // y value           fifo_ptr[3]<<8|fifo_ptr[4];
                        accel_data[2*arr_size + *accel_count] = ( *((int16_t*) (fifo_ptr+5)) );    // z value           fifo_ptr[5]<<8|fifo_ptr[6];
                        poffs+=6;
                        (*accel_count)++;
                }
                if(fifo_ptr[0]&0x20){ // packet contains gyro data
                        gyro_data[             *gyro_count]   = ( *((int16_t*) (fifo_ptr+poffs+1)) ); // x value  fifo_ptr[poffs+1]<<8|fifo_ptr[poffs+2];
                        gyro_data[  arr_size + *gyro_count]   = ( *((int16_t*) (fifo_ptr+poffs+3)) ); // y value  fifo_ptr[poffs+3]<<8|fifo_ptr[poffs+4];
                        gyro_data[2*arr_size + *gyro_count]   = ( *((int16_t*) (fifo_ptr+poffs+5)) ); // z value  fifo_ptr[poffs+5]<<8|fifo_ptr[poffs+6];
                        poffs+=6;
                        (*gyro_count)++;
                }
                if(fifo_ptr[0]&0x10){ // packet contains 20bit extensions, this means it certainly is the 20 byte fifo package type
                        // append 2 bit extension for accelerometer data
                        accel_data[             *accel_count-1] = accel_data[             *accel_count-1]<<2 | fifo_ptr[17]>>6; // x value
                        accel_data[  arr_size + *accel_count-1] = accel_data[  arr_size + *accel_count-1]<<2 | fifo_ptr[18]>>6; // y value
                        accel_data[2*arr_size + *accel_count-1] = accel_data[2*arr_size + *accel_count-1]<<2 | fifo_ptr[19]>>6; // z value
                        // append 3 bit extension for gyro data
                        gyro_data[             *gyro_count-1]   = gyro_data[             *gyro_count-1]<<3 | (fifo_ptr[17]&0x0e)>>1; // x value
                        gyro_data[  arr_size + *gyro_count-1]   = gyro_data[  arr_size + *gyro_count-1]<<3 | (fifo_ptr[18]&0x0e)>>1; // y value
                        gyro_data[2*arr_size + *gyro_count-1]   = gyro_data[2*arr_size + *gyro_count-1]<<3 | (fifo_ptr[19]&0x0e)>>1; // z value
                        poffs=20;
                }
                else if(poffs==12){ // packet is 16 byte
                        poffs=16;
                }
                else{ // packet is 8 byte
                        poffs=8;
                }
                fifo_ptr+=poffs;
        }
}


/*!
 * @brief This API reads the data from the given register address of the sensor.
 */
uint8_t iim42652_reg_read(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct iim42652_dev *dev)
{
        /* Check for null pointer in the device structure*/
        int8_t rslt = iim42652_null_ptr_check(dev);
        /* Proceed if null check is fine */
        if (rslt ==  SENSOR_OK) {
                if (len > 0) {
                        /* If interface selected is SPI */
                        reg_addr = reg_addr | IIM42652_RREG;
                        /* Read the data from the register */
                        rslt = dev->read(dev->spi_dev, reg_addr, reg_data, len);
                }
                else {
                        rslt = SENSOR_E_INVALID_LEN;
                }
        }
        return rslt;
}

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 */
uint8_t iim42652_reg_write (unsigned char write_address, unsigned long data, const struct iim42652_dev *dev)
{
        int8_t rslt;
        uint8_t reg_data[2];

        reg_data[0] = write_address | IIM42652_WREG;
        reg_data[1] = data;

        /* Check for null pointer in the device structure*/
        rslt = iim42652_null_ptr_check(dev);

        /* Check for arguments validity */
        if (rslt == SENSOR_OK) {
             rslt = dev->write(dev->spi_dev, reg_data, 2);
             /* Check for communication error */
             if (rslt != SENSOR_OK)
                     rslt = SENSOR_E_COMM_FAIL;
        }
        return rslt;
}

/*!
 * @brief This API performs the soft reset of the sensor.
 */
uint8_t iim42652_sw_reset(const struct iim42652_dev *dev)
{
        int8_t rslt;
        /* Check for null pointer in the device structure*/
        rslt = iim42652_null_ptr_check(dev);
        /* Proceed if null check is fine */
        if (rslt == SENSOR_OK) {
                rslt = iim42652_reg_write(IIM42652_REG_BANK_SEL, IIM42652_REG_BANK_0, dev);
                if (rslt == SENSOR_OK) {
                        rslt = iim42652_reg_write(IIM42652_DEVICE_CONFIG,0x01, dev);
                        dev->delay_ms(2);
                }
                else {
                        rslt = SENSOR_E_CMD_EXEC_FAILED;
                }
        }
        return rslt;
}

uint8_t iim42652_init(struct iim42652_dev *dev)
{
        int8_t rslt;
        if (dev->settings.high_res_mode) // requires full range operation
        {
                dev->settings.gyro_range = IIM42652_RANGE_PM2kdps;
                dev->settings.accel_range = IIM42652_RANGE_PM16G;
                rslt = SENSOR_W_INVALID_CONFIG;
        }
        if (dev->settings.gyro_odr >= IIM42652_ODR_6_25HZ) // this checks if gyro odr is lower than allowed
        {
                dev->settings.gyro_odr = IIM42652_ODR_12_5HZ;
                rslt = SENSOR_W_INVALID_CONFIG;
        }

        rslt |= iim42652_sw_reset(dev);  // also sets register bank to 0 and does NULL ptr check
        if (rslt != SENSOR_OK) {
                return rslt;
        }
        if(dev->settings.gyro_en) // set gyro odr and scale
        {
                rslt |= iim42652_reg_write(IIM42652_GYRO_CONFIG0, dev->settings.gyro_odr<<5 | dev->settings.gyro_odr, dev);
        }
        if(dev->settings.accel_en) // set accelerometer odr and scale
        {
                rslt |= iim42652_reg_write(IIM42652_ACCEL_CONFIG0, dev->settings.accel_range<<5 | dev->settings.accel_odr, dev);
        }
        if(dev->settings.fifo_mode != IIM42652_FIFO_BYPASS)
        {
                rslt |= iim42652_reg_write(IIM42652_FIFO_CONFIG, dev->settings.fifo_mode<<6, dev); // enable stream-to-fifo
                rslt |= iim42652_reg_write(IIM42652_FIFO_CONFIG1, 0x40|dev->settings.high_res_mode<<4 |dev->settings.temp_en<<2|dev->settings.gyro_en<<1|dev->settings.accel_en, dev); // enable high-res mode, and partial fifo readouts
        }
        if(dev->settings.use_ext_clk)
        {
                rslt |= iim42652_reg_write(IIM42652_REG_BANK_SEL, IIM42652_REG_BANK_1, dev);
                rslt |= iim42652_reg_write(IIM42652_INTF_CONFIG5, 0x04, dev); // set pin 9 to clkin
                rslt |= iim42652_reg_write(IIM42652_REG_BANK_SEL, IIM42652_REG_BANK_0, dev);

                uint8_t setting = 0x00;
                rslt |= iim42652_reg_read(IIM42652_INTF_CONFIG1, &setting, 1, dev);
                setting &= 0xf0; // mask non-reserved bits
                setting |= 0x05; // use external clock
                rslt |= iim42652_reg_write(IIM42652_INTF_CONFIG1, setting, dev); // set pin 9 to clkin
        }
        rslt |= iim42652_reg_write(IIM42652_INTF_CONFIG0, 0x00, dev); // change endian format to access data and fifo size with pointers directly
        if (rslt != SENSOR_OK){
                rslt = SENSOR_E_CMD_EXEC_FAILED;
        }
        return rslt;
}

uint8_t iim42652_get_id(uint8_t *id, const struct iim42652_dev *dev) // returns size in bytes unless otherwise configured in FIFO_COUNT_REC
{
        uint8_t rslt;
        rslt = iim42652_reg_read(IIM42652_WHO_AM_I, id, 1, dev);
        return rslt;
}

uint8_t iim42652_fifo_size(uint16_t *fifo_size, const struct iim42652_dev *dev) // returns size in bytes unless otherwise configured in FIFO_COUNT_REC
{
        uint8_t rslt;
        rslt = iim42652_reg_read(IIM42652_FIFO_COUNTH, (uint8_t*)fifo_size, 2, dev);
        //*fifo_size = (*fifo_size)>>8 | ((*fifo_size)&0xff)<<8; // swapping bytes, could be done by endian setting TODO
        return rslt;
}

uint8_t iim42652_read_fifo(uint8_t* data, uint16_t *fifo_size, const struct iim42652_dev *dev)
{
        uint8_t rslt;
        rslt = iim42652_fifo_size(fifo_size, dev); // also switches to reg bank 0
        if (rslt == SENSOR_OK) {
                rslt = iim42652_reg_read(IIM42652_FIFO_DATA, data, *fifo_size, dev);
        }
        return rslt;
}


uint8_t iim42652_get_sensor_data(int32_t* gyro_data, uint8_t* gyro_count, int32_t* accel_data, uint8_t* accel_count, const uint8_t arr_size, const struct iim42652_dev *dev)
{
        uint8_t iim42652_fifo_data[2048];
        uint16_t fifo_size;
        uint8_t rslt = iim42652_read_fifo(iim42652_fifo_data, &fifo_size, dev);
        if (rslt == SENSOR_OK){
                decode_fifo_data(iim42652_fifo_data, fifo_size, gyro_data, gyro_count, accel_data, accel_count, arr_size);
        }
        return rslt;
}

uint8_t iim42652_enable(const struct iim42652_dev *dev)
{
	uint8_t setting = dev->settings.temp_en<<5;
	if(dev->settings.gyro_en){
		setting |= 0x0c; // LN mode
	}
	if(dev->settings.accel_en){
		setting |= 0x03; // LN mode
	}
	return iim42652_reg_write(IIM42652_PWR_MGMT0, setting, dev);
}


uint8_t iim42652_disable(const struct iim42652_dev *dev)
{
        return iim42652_reg_write(IIM42652_PWR_MGMT0, 0x00, dev);
}
