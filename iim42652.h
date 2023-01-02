/**\mainpage
 * Copyright (C) 2022 SIP Lab ETH Zurich
 *
 * File         iim42652.h
 * Date         29.12.2022
 * Author       Manuel Meier
 * Version      1.0.0
 *
 */

#ifndef IIM42652_H_
#define IIM42652_H_

/* Header includes */
#include "iim42652_defs.h"

/*! CPP guard */
#ifdef __cplusplus
extern "C"
{
#endif


uint8_t iim42652_reg_read(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct iim42652_dev *dev);

uint8_t iim42652_reg_write (unsigned char write_address, unsigned long data, const struct iim42652_dev *dev);

uint8_t iim42652_sw_reset(const struct iim42652_dev *dev);

uint8_t iim42652_init(struct iim42652_dev *dev);

uint8_t iim42652_get_id(uint8_t *id, const struct iim42652_dev *dev);

uint8_t iim42652_fifo_size(uint16_t *fifo_size, const struct iim42652_dev *dev);

uint8_t iim42652_read_fifo(uint8_t* data, uint16_t *fifo_size, const struct iim42652_dev *dev);

uint8_t iim42652_get_sensor_data(int32_t* gyro_data, uint8_t* gyro_count, int32_t* accel_data, uint8_t* accel_count, const uint8_t arr_size, const struct iim42652_dev *dev);

uint8_t iim42652_enable(const struct iim42652_dev *dev);

uint8_t iim42652_disable(const struct iim42652_dev *dev);


#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* IIM42652_H_ */
/** @}*/

