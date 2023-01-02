# IIM42652-Sensor-API
Basic functions to use IIM42652 IMU.
The repository includes iim42652.h, iim42652.c and iim42652_defs.h files.

## Integration details

- Integrate iim42652.h, iim42652_defs.h and iim42652.c file in to your project.
- Include the iim42652.h file in your code like below.

```c
#include "iim42652.h"
```

## Current state of the library
This library is - in its current state - limited in functionality and only includes basic functionality. If you are interested in more features, please file issues/pull requests.

### Currently supported configurations (limitations)
- SPI 4-wire (only)
- FIFO mode (only)
- High-Res or Normal mode
- All range and ODR settings
- Usage of internal or external clock
- Accelerometer and Gyro processing only (Temperature measurements are currently not decoded/used)

## Use
To initialize the sensor, you will first need to create a device structure. You 
can do this by creating an instance of the structure iim42652_dev. This includes assigning function pointers to spi read and write functions as well as a void pointer to a structure which is passed to the spi read and write function (e.g., containing CS pin configuration).

Example usage code:

```c
#include "iim42652.h"
...
int main(void){
...
  int32_t iim42652_accel_data[3][102]; // array size (2nd dim) has to be passed to fifo read function
  int32_t iim42652_gyro_data[3][102];  
  uint8_t iim42652_gyro_sampleCnt = 0;
  uint8_t iim42652_accel_sampleCnt = 0;
  struct iim42652_dev iim42652;

  iim42652.read = read_spi;
  iim42652.write = write_spi;
  iim42652.delay_ms = wait_ms;
  iim42652.spi_dev = IIM42652_SPI_DEVICE;
  iim42652.settings.fifo_mode = IIM42652_STREAM_TO_FIFO;
  iim42652.settings.temp_en = false;
  iim42652.settings.gyro_en = true;
  iim42652.settings.accel_en = true;
  iim42652.settings.gyro_odr = IIM42652_ODR_200HZ;  
  iim42652.settings.accel_odr = IIM42652_ODR_200HZ;
  iim42652.settings.gyro_range = IIM42652_RANGE_PM2kdps;
  iim42652.settings.accel_range = IIM42652_RANGE_PM16G;
  iim42652.settings.high_res_mode = true; // overrides range to max range if true
  iim42652.settings.use_ext_clk = true;

  uint8_t id;
  uint8_t result = iim42652_get_id(&id, &iim42652);
  if(id != 0x6f || result != SENSOR_OK){ //check for hw-defined bit pattern
    printf("\n\r\u274C IIM42652 (gyro/accelerometer) (ID Register: 0x%02x, should be: 0x%02x), return code: 0x%02x", id, 0x6f, result);
  }
  else{
    iim42652_init(&iim42652);
    iim42652_enable(&iim42652);
  }
...

  iim42652_get_sensor_data(iim42652_gyro_data, &iim42652_gyro_sampleCnt, iim42652_accel_data, &iim42652_accel_sampleCnt, 102, &iim42652);


```
