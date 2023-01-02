/**
 * Copyright (C) 2022 SIP Lab ETH Zurich
 *
 * File         iim42652_defs.h
 * Date         29.12.2022
 * Author       Manuel Meier
 * Version      1.0.0
 *
 */

#ifndef IIM42652_DEFS_H_
#define IIM42652_DEFS_H_

/*! CPP guard */
#ifdef __cplusplus
extern "C"
{
#endif

/********************************************************/
/* header includes */
#include <stdint.h>
#include <stddef.h>
#include "platform_devices.h"

/*************************** Common macros   *****************************/

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)       S8_C(x)
#define UINT8_C(x)      U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)      S16_C(x)
#define UINT16_C(x)     U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)      S32_C(x)
#define UINT32_C(x)     U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)      S64_C(x)
#define UINT64_C(x)     U64_C(x)
#endif

/**@}*/

/**\name C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL   0
#else
#define NULL   ((void *) 0)
#endif
#endif

#ifndef TRUE
#define TRUE                UINT8_C(1)
#endif

#ifndef FALSE
#define FALSE               UINT8_C(0)
#endif


/********************************************************/
/** Specific definitions */

// SPI Read and Write Mask
#define   IIM42652_WREG                         UINT8_C(0x00)
#define   IIM42652_RREG                         UINT8_C(0x80)

/**REGISTERS**************************/
// Registers bank 0
#define   IIM42652_DEVICE_CONFIG                UINT8_C(0x11)
#define   IIM42652_DRIVE_CONFIG                 UINT8_C(0x13)
#define   IIM42652_INT_CONFIG                   UINT8_C(0x14)
#define   IIM42652_FIFO_CONFIG                  UINT8_C(0x16)
#define   IIM42652_TEMP_DATA1_UI                UINT8_C(0x1D)
#define   IIM42652_TEMP_DATA0_UI                UINT8_C(0x1E)
#define   IIM42652_ACCEL_DATA_X1_UI             UINT8_C(0x1F)
#define   IIM42652_ACCEL_DATA_X0_UI             UINT8_C(0x20)
#define   IIM42652_ACCEL_DATA_Y1_UI             UINT8_C(0x21)
#define   IIM42652_ACCEL_DATA_Y0_UI             UINT8_C(0x22)
#define   IIM42652_ACCEL_DATA_Z1_UI             UINT8_C(0x23)
#define   IIM42652_ACCEL_DATA_Z0_UI             UINT8_C(0x24)
#define   IIM42652_GYRO_DATA_X1_UI              UINT8_C(0x25)
#define   IIM42652_GYRO_DATA_X0_UI              UINT8_C(0x26)
#define   IIM42652_GYRO_DATA_Y1_UI              UINT8_C(0x27)
#define   IIM42652_GYRO_DATA_Y0_UI              UINT8_C(0x28)
#define   IIM42652_GYRO_DATA_Z1_UI              UINT8_C(0x29)
#define   IIM42652_GYRO_DATA_Z0_UI              UINT8_C(0x2A)
#define   IIM42652_TMST_FSYNCH                  UINT8_C(0x2B)
#define   IIM42652_TMST_FSYNCL                  UINT8_C(0x2C)
#define   IIM42652_INT_STATUS                   UINT8_C(0x2D)
#define   IIM42652_FIFO_COUNTH                  UINT8_C(0x2E)
#define   IIM42652_FIFO_COUNTL                  UINT8_C(0x2F)
#define   IIM42652_FIFO_DATA                    UINT8_C(0x30)
#define   IIM42652_APEX_DATA0                   UINT8_C(0x31)
#define   IIM42652_APEX_DATA1                   UINT8_C(0x32)
#define   IIM42652_APEX_DATA2                   UINT8_C(0x33)
#define   IIM42652_APEX_DATA3                   UINT8_C(0x34)
#define   IIM42652_APEX_DATA4                   UINT8_C(0x35)
#define   IIM42652_APEX_DATA5                   UINT8_C(0x36)
#define   IIM42652_INT_STATUS2                  UINT8_C(0x37)
#define   IIM42652_INT_STATUS3                  UINT8_C(0x38)
#define   IIM42652_SIGNAL_PATH_RESET            UINT8_C(0x4B)
#define   IIM42652_INTF_CONFIG0                 UINT8_C(0x4C)
#define   IIM42652_INTF_CONFIG1                 UINT8_C(0x4D)
#define   IIM42652_PWR_MGMT0                    UINT8_C(0x4E)
#define   IIM42652_GYRO_CONFIG0                 UINT8_C(0x4F)
#define   IIM42652_ACCEL_CONFIG0                UINT8_C(0x50)
#define   IIM42652_GYRO_CONFIG1                 UINT8_C(0x51)
#define   IIM42652_GYRO_ACCEL_CONFIG0           UINT8_C(0x52)
#define   IIM42652_ACCEL_CONFIG1                UINT8_C(0x53)
#define   IIM42652_TMST_CONFIG                  UINT8_C(0x54)
#define   IIM42652_APEX_CONFIG0                 UINT8_C(0x56)
#define   IIM42652_SMD_CONFIG                   UINT8_C(0x57)
#define   IIM42652_FIFO_CONFIG1                 UINT8_C(0x5F)
#define   IIM42652_FIFO_CONFIG2                 UINT8_C(0x60)
#define   IIM42652_FIFO_CONFIG3                 UINT8_C(0x61)
#define   IIM42652_FSYNC_CONFIG                 UINT8_C(0x62)
#define   IIM42652_INT_CONFIG0                  UINT8_C(0x63)
#define   IIM42652_INT_CONFIG1                  UINT8_C(0x64)
#define   IIM42652_INT_SOURCE0                  UINT8_C(0x65)
#define   IIM42652_INT_SOURCE1                  UINT8_C(0x66)
#define   IIM42652_INT_SOURCE3                  UINT8_C(0x68)
#define   IIM42652_INT_SOURCE4                  UINT8_C(0x69)
#define   IIM42652_FIFO_LOST_PKT0               UINT8_C(0x6C)
#define   IIM42652_FIFO_LOST_PKT1               UINT8_C(0x6D)
#define   IIM42652_SELF_TEST_CONFIG             UINT8_C(0x70)
#define   IIM42652_WHO_AM_I                     UINT8_C(0x75)
#define   IIM42652_REG_BANK_SEL                 UINT8_C(0x76)

// Registers bank 1
#define   IIM42652_SENSOR_CONFIG0               UINT8_C(0x03)
#define   IIM42652_GYRO_CONFIG_STATIC2          UINT8_C(0x0B)
#define   IIM42652_GYRO_CONFIG_STATIC3          UINT8_C(0x0C)
#define   IIM42652_GYRO_CONFIG_STATIC4          UINT8_C(0x0D)
#define   IIM42652_GYRO_CONFIG_STATIC5          UINT8_C(0x0E)
#define   IIM42652_GYRO_CONFIG_STATIC6          UINT8_C(0x0F)
#define   IIM42652_GYRO_CONFIG_STATIC7          UINT8_C(0x10)
#define   IIM42652_GYRO_CONFIG_STATIC8          UINT8_C(0x11)
#define   IIM42652_GYRO_CONFIG_STATIC9          UINT8_C(0x12)
#define   IIM42652_GYRO_CONFIG_STATIC10         UINT8_C(0x13)
#define   IIM42652_XG_ST_DATA                   UINT8_C(0x5F)
#define   IIM42652_YG_ST_DATA                   UINT8_C(0x60)
#define   IIM42652_ZG_ST_DATA                   UINT8_C(0x61)
#define   IIM42652_TMSTVAL0                     UINT8_C(0x62)
#define   IIM42652_TMSTVAL1                     UINT8_C(0x63)
#define   IIM42652_TMSTVAL2                     UINT8_C(0x64)
#define   IIM42652_INTF_CONFIG4                 UINT8_C(0x7A)
#define   IIM42652_INTF_CONFIG5                 UINT8_C(0x7B)
#define   IIM42652_INTF_CONFIG6                 UINT8_C(0x7C)

// Registers bank 2
#define   IIM42652_ACCEL_CONFIG_STATIC2         UINT8_C(0x03)
#define   IIM42652_ACCEL_CONFIG_STATIC3         UINT8_C(0x04)
#define   IIM42652_ACCEL_CONFIG_STATIC4         UINT8_C(0x05)
#define   IIM42652_XA_ST_DATA                   UINT8_C(0x3B)
#define   IIM42652_YA_ST_DATA                   UINT8_C(0x3C)
#define   IIM42652_ZA_ST_DATA                   UINT8_C(0x3D)

// Registers bank 3
#define   IIM42652_PU_PD_CONFIG1                UINT8_C(0x06)
#define   IIM42652_PU_PD_CONFIG2                UINT8_C(0x0E)

// Registers bank 4
#define   IIM42652_FDR_CONFIG                   UINT8_C(0x09)
#define   IIM42652_APEX_CONFIG1                 UINT8_C(0x40)
#define   IIM42652_APEX_CONFIG2                 UINT8_C(0x41)
#define   IIM42652_APEX_CONFIG3                 UINT8_C(0x42)
#define   IIM42652_APEX_CONFIG4                 UINT8_C(0x43)
#define   IIM42652_APEX_CONFIG5                 UINT8_C(0x44)
#define   IIM42652_APEX_CONFIG6                 UINT8_C(0x45)
#define   IIM42652_APEX_CONFIG7                 UINT8_C(0x46)
#define   IIM42652_APEX_CONFIG8                 UINT8_C(0x47)
#define   IIM42652_APEX_CONFIG9                 UINT8_C(0x48)
#define   IIM42652_APEX_CONFIG10                UINT8_C(0x49)
#define   IIM42652_ACCEL_WOM_X_THR              UINT8_C(0x4A)
#define   IIM42652_ACCEL_WOM_Y_THR              UINT8_C(0x4B)
#define   IIM42652_ACCEL_WOM_Z_THR              UINT8_C(0x4C)
#define   IIM42652_INT_SOURCE6                  UINT8_C(0x4D)
#define   IIM42652_INT_SOURCE7                  UINT8_C(0x4E)
#define   IIM42652_INT_SOURCE8                  UINT8_C(0x4F)
#define   IIM42652_INT_SOURCE9                  UINT8_C(0x50)
#define   IIM42652_INT_SOURCE10                 UINT8_C(0x51)
#define   IIM42652_OFFSET_USER0                 UINT8_C(0x77)
#define   IIM42652_OFFSET_USER1                 UINT8_C(0x78)
#define   IIM42652_OFFSET_USER2                 UINT8_C(0x79)
#define   IIM42652_OFFSET_USER3                 UINT8_C(0x7A)
#define   IIM42652_OFFSET_USER4                 UINT8_C(0x7B)
#define   IIM42652_OFFSET_USER5                 UINT8_C(0x7C)
#define   IIM42652_OFFSET_USER6                 UINT8_C(0x7D)
#define   IIM42652_OFFSET_USER7                 UINT8_C(0x7E)
#define   IIM42652_OFFSET_USER8                 UINT8_C(0x7F)

/**REGISTER SETTINGS**************************/
// register banks
#define   IIM42652_REG_BANK_0                   UINT8_C(0x00)
#define   IIM42652_REG_BANK_1                   UINT8_C(0x01)
#define   IIM42652_REG_BANK_2                   UINT8_C(0x02)
#define   IIM42652_REG_BANK_3                   UINT8_C(0x03)
#define   IIM42652_REG_BANK_4                   UINT8_C(0x04)
// ODR
#define   IIM42652_ODR_32KHZ                    UINT8_C(0x01)
#define   IIM42652_ODR_16KHZ                    UINT8_C(0x02)
#define   IIM42652_ODR_8KHZ                     UINT8_C(0x03)
#define   IIM42652_ODR_4KHZ                     UINT8_C(0x04)
#define   IIM42652_ODR_2KHZ                     UINT8_C(0x05)
#define   IIM42652_ODR_1KHZ                     UINT8_C(0x06)
#define   IIM42652_ODR_500HZ                    UINT8_C(0x0F)
#define   IIM42652_ODR_200HZ                    UINT8_C(0x07)
#define   IIM42652_ODR_100HZ                    UINT8_C(0x08)
#define   IIM42652_ODR_50HZ                     UINT8_C(0x09)
#define   IIM42652_ODR_25HZ                     UINT8_C(0x0A)
#define   IIM42652_ODR_12_5HZ                   UINT8_C(0x0B)
#define   IIM42652_ODR_6_25HZ                   UINT8_C(0x0C)   // only available for accelerometer, not for gyro
#define   IIM42652_ODR_3_125HZ                  UINT8_C(0x0D)   // only available for accelerometer, not for gyro
#define   IIM42652_ODR_1_5625HZ                 UINT8_C(0x0E)   // only available for accelerometer, not for gyro

// range accelerometer
#define   IIM42652_RANGE_PM16G                  UINT8_C(0x00)
#define   IIM42652_RANGE_PM8G                   UINT8_C(0x01)
#define   IIM42652_RANGE_PM4G                   UINT8_C(0x02)
#define   IIM42652_RANGE_PM2G                   UINT8_C(0x03)

// range gyro
#define   IIM42652_RANGE_PM2kdps                UINT8_C(0x00)
#define   IIM42652_RANGE_PM1kdps                UINT8_C(0x01)
#define   IIM42652_RANGE_PM500dps               UINT8_C(0x02)
#define   IIM42652_RANGE_PM250dps               UINT8_C(0x03)
#define   IIM42652_RANGE_PM125dps               UINT8_C(0x04)
#define   IIM42652_RANGE_PM62_5dps              UINT8_C(0x05)
#define   IIM42652_RANGE_PM31_25dps             UINT8_C(0x06)
#define   IIM42652_RANGE_PM15_625dps            UINT8_C(0x07)

// fifo mode
#define   IIM42652_FIFO_BYPASS                  UINT8_C(0x00)
#define   IIM42652_STREAM_TO_FIFO               UINT8_C(0x01)
#define   IIM42652_STREAM_TO_FIFO_STOP_ON_FULL  UINT8_C(0x10)



// Status codes
/**\name API success code */
#define SENSOR_OK                               INT8_C(0)
/**\name API error codes */
#define SENSOR_E_NULL_PTR                       INT8_C(-1)
#define SENSOR_E_CMD_EXEC_FAILED                INT8_C(-2)
#define SENSOR_E_INVALID_LEN                    INT8_C(-3)
#define SENSOR_E_COMM_FAIL                      INT8_C(-4)

/**\name API warning codes */
#define SENSOR_W_INVALID_CONFIG                 UINT8_C(1)


/********************************************************/
/*!
 * @brief Type definitions
 */
typedef int8_t (*iim42652_com_fptr_t)(spi_device dev, uint8_t reg_addr,
                uint8_t *data, uint16_t len);

typedef int8_t (*iim42652_wrt_fptr_t)(spi_device dev,
                uint8_t *data, uint16_t len);

typedef void (*iim42652_delay_fptr_t)(uint32_t period);

/*!
 * @brief iim42652 device settings structure
 */
struct iim42652_settings
{
    bool temp_en;
    bool gyro_en;
    bool accel_en;
    bool high_res_mode;
    bool use_ext_clk;

    uint8_t fifo_mode;

    uint8_t gyro_odr;
    uint8_t accel_odr;
    uint8_t gyro_range;
    uint8_t accel_range;

};


/*!
 * @brief iim42652 device structure
 */
struct iim42652_dev {
        /*! Read function pointer */
        iim42652_com_fptr_t read;
        /*! Write function pointer */
        iim42652_wrt_fptr_t write;
        /*! Delay function pointer */
        iim42652_delay_fptr_t delay_ms;
        spi_device spi_dev;

        /*! Sensor settings */
        struct iim42652_settings settings;
};

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* IIM42652_DEFS_H_ */
/** @}*/
/** @}*/
