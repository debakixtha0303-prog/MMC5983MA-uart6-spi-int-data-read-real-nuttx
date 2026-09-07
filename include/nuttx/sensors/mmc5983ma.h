#ifndef __INCLUDE_NUTTX_SENSORS_MMC5983MA_H
#define __INCLUDE_NUTTX_SENSORS_MMC5983MA_H

#include <nuttx/config.h>
#include <stdint.h>
#include <nuttx/compiler.h>
#include <nuttx/spi/spi.h>

/* Registers */

#define MMC5983MA_REG_XOUT_0       0x00
#define MMC5983MA_REG_XOUT_1       0x01
#define MMC5983MA_REG_YOUT_0       0x02
#define MMC5983MA_REG_YOUT_1       0x03
#define MMC5983MA_REG_ZOUT_0       0x04
#define MMC5983MA_REG_ZOUT_1       0x05
#define MMC5983MA_REG_XYZOUT_2     0x06
#define MMC5983MA_REG_STATUS       0x08
#define MMC5983MA_REG_CONTROL_0    0x09
#define MMC5983MA_REG_CONTROL_1    0x0A
#define MMC5983MA_REG_CONTROL_2    0x0B
#define MMC5983MA_REG_CONTROL_3    0x0C
#define MMC5983MA_REG_PRODUCT_ID   0x2F

#define MMC5983MA_PRODUCT_ID_VAL   0x30

/* Status */

#define MMC5983MA_STATUS_MEAS_M_DONE   (1 << 0)

/* Control 0 */

#define MMC5983MA_CTRL0_TM_M       (1 << 0)

/* Control 1 */

#define MMC5983MA_CTRL1_SW_RST     (1 << 7)

/* Conversion */

#define MMC5983MA_ZERO              131072.0f
#define MMC5983MA_COUNTS_PER_G      16384.0f

#define MMC5983MA_G_TO_UT           100.0f
#define MMC5983MA_G_TO_T            1.0e-4f

typedef struct
{
  FAR struct spi_dev_s *spi;
  int dev_id;
} mmc5983ma_dev_t;

typedef struct
{
  uint32_t raw_x;
  uint32_t raw_y;
  uint32_t raw_z;

  float x_gauss;
  float y_gauss;
  float z_gauss;

  float x_uT;
  float y_uT;
  float z_uT;

  float x_T;
  float y_T;
  float z_T;
} mmc5983ma_data_t;

int mmc5983ma_init(FAR mmc5983ma_dev_t *dev,
                   FAR struct spi_dev_s *spi,
                   int dev_id);

int mmc5983ma_read_reg(FAR mmc5983ma_dev_t *dev,
                       uint8_t reg,
                       FAR uint8_t *val);

int mmc5983ma_write_reg(FAR mmc5983ma_dev_t *dev,
                        uint8_t reg,
                        uint8_t val);

int mmc5983ma_read_mag(FAR mmc5983ma_dev_t *dev,
                       FAR mmc5983ma_data_t *data);

#endif