/****************************************************************************
 * drivers/sensors/mmc5983ma.c
 *
 * MMC5983MA 3-Axis Magnetometer Driver
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#include <nuttx/spi/spi.h>
#include <nuttx/sensors/mmc5983ma.h>

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

#define MMC5983MA_SPI_FREQ        1000000
#define MMC5983MA_TIMEOUT_MS      50

/****************************************************************************
 * Read Register
 ****************************************************************************/

int mmc5983ma_read_reg(FAR mmc5983ma_dev_t *dev,
                       uint8_t reg,
                       FAR uint8_t *val)
{
  uint8_t tx[2];
  uint8_t rx[2];

  if (dev == NULL || dev->spi == NULL || val == NULL)
    {
      return -EINVAL;
    }

  memset(tx, 0, sizeof(tx));
  memset(rx, 0, sizeof(rx));

  tx[0] = reg | 0x80;
  tx[1] = 0x00;

  SPI_LOCK(dev->spi, true);

  SPI_SETMODE(dev->spi, SPIDEV_MODE0);
  SPI_SETBITS(dev->spi, 8);
  SPI_SETFREQUENCY(dev->spi, MMC5983MA_SPI_FREQ);

  SPI_SELECT(dev->spi, dev->dev_id, true);

  SPI_EXCHANGE(dev->spi, tx, rx, 2);

  SPI_SELECT(dev->spi, dev->dev_id, false);

  SPI_LOCK(dev->spi, false);

  *val = rx[1];

  return OK;
}

/****************************************************************************
 * Write Register
 ****************************************************************************/

int mmc5983ma_write_reg(FAR mmc5983ma_dev_t *dev,
                        uint8_t reg,
                        uint8_t val)
{
  uint8_t tx[2];
  uint8_t rx[2];

  if (dev == NULL || dev->spi == NULL)
    {
      return -EINVAL;
    }

  memset(rx, 0, sizeof(rx));

  tx[0] = reg & 0x7f;
  tx[1] = val;

  SPI_LOCK(dev->spi, true);

  SPI_SETMODE(dev->spi, SPIDEV_MODE0);
  SPI_SETBITS(dev->spi, 8);
  SPI_SETFREQUENCY(dev->spi, MMC5983MA_SPI_FREQ);

  SPI_SELECT(dev->spi, dev->dev_id, true);

  SPI_EXCHANGE(dev->spi, tx, rx, 2);

  SPI_SELECT(dev->spi, dev->dev_id, false);

  SPI_LOCK(dev->spi, false);

  return OK;
}

/****************************************************************************
 * Initialize
 ****************************************************************************/

int mmc5983ma_init(FAR mmc5983ma_dev_t *dev,
                   FAR struct spi_dev_s *spi,
                   int dev_id)
{
  uint8_t product_id;
  int ret;

  if (dev == NULL || spi == NULL)
    {
      return -EINVAL;
    }

  dev->spi = spi;
  dev->dev_id = dev_id;

  printf("[MMC5983MA] Initializing...\n");

  /*
   * SPI configuration
   */

  SPI_LOCK(dev->spi, true);

  SPI_SETMODE(dev->spi, SPIDEV_MODE0);
  SPI_SETBITS(dev->spi, 8);
  SPI_SETFREQUENCY(dev->spi, MMC5983MA_SPI_FREQ);

  SPI_LOCK(dev->spi, false);

  /*
   * Read Product ID
   */

  ret = mmc5983ma_read_reg(dev,
                           MMC5983MA_REG_PRODUCT_ID,
                           &product_id);

  if (ret < 0)
    {
      printf("[MMC5983MA] PRODUCT_ID READ FAILED: %d\n", ret);
      return ret;
    }

  printf("[MMC5983MA] PRODUCT_ID = 0x%02X\n",
         product_id);

  if (product_id != MMC5983MA_PRODUCT_ID_VAL)
    {
      printf("[MMC5983MA] WRONG PRODUCT ID: 0x%02X\n",
             product_id);

      return -ENODEV;
    }

  /*
   * Software reset
   */

  ret = mmc5983ma_write_reg(dev,
                            MMC5983MA_REG_CONTROL_1,
                            MMC5983MA_CTRL1_SW_RST);

  if (ret < 0)
    {
      return ret;
    }

  usleep(20000);

  /*
   * Default configuration
   */

  ret = mmc5983ma_write_reg(dev,
                            MMC5983MA_REG_CONTROL_0,
                            0x00);

  if (ret < 0)
    {
      return ret;
    }

  ret = mmc5983ma_write_reg(dev,
                            MMC5983MA_REG_CONTROL_1,
                            0x00);

  if (ret < 0)
    {
      return ret;
    }

  printf("[MMC5983MA] INIT OK\n");

  return OK;
}

/****************************************************************************
 * Read Magnetic Field
 ****************************************************************************/

int mmc5983ma_read_mag(FAR mmc5983ma_dev_t *dev,
                       FAR mmc5983ma_data_t *data)
{
  uint8_t ctrl0;
  uint8_t status;

  uint8_t tx[8];
  uint8_t rx[8];

  uint32_t raw_x;
  uint32_t raw_y;
  uint32_t raw_z;

  int ret;
  int elapsed;

  if (dev == NULL || dev->spi == NULL || data == NULL)
    {
      return -EINVAL;
    }

  memset(data, 0, sizeof(*data));

  /**************************************************************************
   * Read CONTROL0
   **************************************************************************/

  ret = mmc5983ma_read_reg(dev,
                           MMC5983MA_REG_CONTROL_0,
                           &ctrl0);

  if (ret < 0)
    {
      return ret;
    }

  /**************************************************************************
   * Start magnetic measurement
   **************************************************************************/

  ctrl0 |= MMC5983MA_CTRL0_TM_M;

  ret = mmc5983ma_write_reg(dev,
                            MMC5983MA_REG_CONTROL_0,
                            ctrl0);

  if (ret < 0)
    {
      return ret;
    }

  /**************************************************************************
   * Wait for measurement complete
   **************************************************************************/

  for (elapsed = 0;
       elapsed < MMC5983MA_TIMEOUT_MS;
       elapsed++)
    {
      usleep(1000);

      ret = mmc5983ma_read_reg(dev,
                               MMC5983MA_REG_STATUS,
                               &status);

      if (ret < 0)
        {
          return ret;
        }

      if (status & MMC5983MA_STATUS_MEAS_M_DONE)
        {
          break;
        }
    }

  if ((status & MMC5983MA_STATUS_MEAS_M_DONE) == 0)
    {
      printf("[MMC5983MA] MEASUREMENT TIMEOUT "
             "STATUS=0x%02X\n",
             status);

      return -ETIMEDOUT;
    }

  /**************************************************************************
   * Burst read X/Y/Z
   **************************************************************************/

  memset(tx, 0, sizeof(tx));
  memset(rx, 0, sizeof(rx));

  tx[0] = MMC5983MA_REG_XOUT_0 | 0x80;

  SPI_LOCK(dev->spi, true);

  SPI_SETMODE(dev->spi, SPIDEV_MODE0);
  SPI_SETBITS(dev->spi, 8);
  SPI_SETFREQUENCY(dev->spi, MMC5983MA_SPI_FREQ);

  SPI_SELECT(dev->spi, dev->dev_id, true);

  SPI_EXCHANGE(dev->spi,
               tx,
               rx,
               sizeof(tx));

  SPI_SELECT(dev->spi, dev->dev_id, false);

  SPI_LOCK(dev->spi, false);

  /**************************************************************************
   * Decode 18-bit raw values
   **************************************************************************/

  raw_x = ((uint32_t)rx[1] << 10) |
          ((uint32_t)rx[2] << 2)  |
          ((rx[7] >> 6) & 0x03);

  raw_y = ((uint32_t)rx[3] << 10) |
          ((uint32_t)rx[4] << 2)  |
          ((rx[7] >> 4) & 0x03);

  raw_z = ((uint32_t)rx[5] << 10) |
          ((uint32_t)rx[6] << 2)  |
          ((rx[7] >> 2) & 0x03);

  data->raw_x = raw_x;
  data->raw_y = raw_y;
  data->raw_z = raw_z;

  /**************************************************************************
   * Magnetic Field Conversion (Fixed-Point Integer Math)
   *
   * 18-bit Mode Offset = 131072 (2^17)
   * Sensitivity        = 16384 LSB / Gauss
   **************************************************************************/

  int32_t dx = (int32_t)raw_x - 131072;
  int32_t dy = (int32_t)raw_y - 131072;
  int32_t dz = (int32_t)raw_z - 131072;

  /* Scale microTesla (uT) by 100 to print 2 decimal places */
  int32_t xuT_x100 = (dx * 10000) / 16384;
  int32_t yuT_x100 = (dy * 10000) / 16384;
  int32_t zuT_x100 = (dz * 10000) / 16384;

  /* Scale Gauss by 10000 to print 4 decimal places */
  int32_t xG_x10000 = (dx * 10000) / 16384;
  int32_t yG_x10000 = (dy * 10000) / 16384;
  int32_t zG_x10000 = (dz * 10000) / 16384;

  /* Nanotesla calculations for high precision Tesla output */
  int64_t xnT = ((int64_t)dx * 100000000LL) / 16384;
  int64_t ynT = ((int64_t)dy * 100000000LL) / 16384;
  int64_t znT = ((int64_t)dz * 100000000LL) / 16384;
/**************************************************************************
   * Real Magnetic Field Outputs
   **************************************************************************/

  printf("[MMC5983MA] FIELD uT: X=%s%ld.%02ld, Y=%s%ld.%02ld, Z=%s%ld.%02ld uT\n",
         xuT_x100 < 0 ? "-" : "",
         (long)(xuT_x100 < 0 ? -xuT_x100 : xuT_x100) / 100,
         (long)(xuT_x100 < 0 ? -xuT_x100 : xuT_x100) % 100,
         yuT_x100 < 0 ? "-" : "",
         (long)(yuT_x100 < 0 ? -yuT_x100 : yuT_x100) / 100,
         (long)(yuT_x100 < 0 ? -yuT_x100 : yuT_x100) % 100,
         zuT_x100 < 0 ? "-" : "",
         (long)(zuT_x100 < 0 ? -zuT_x100 : zuT_x100) / 100,
         (long)(zuT_x100 < 0 ? -zuT_x100 : zuT_x100) % 100);

  printf("[MMC5983MA] FIELD Gauss: X=%s%ld.%04ld, Y=%s%ld.%04ld, Z=%s%ld.%04ld G\n",
         xG_x10000 < 0 ? "-" : "",
         (long)(xG_x10000 < 0 ? -xG_x10000 : xG_x10000) / 10000,
         (long)(xG_x10000 < 0 ? -xG_x10000 : xG_x10000) % 10000,
         yG_x10000 < 0 ? "-" : "",
         (long)(yG_x10000 < 0 ? -yG_x10000 : yG_x10000) / 10000,
         (long)(yG_x10000 < 0 ? -yG_x10000 : yG_x10000) % 10000,
         zG_x10000 < 0 ? "-" : "",
         (long)(zG_x10000 < 0 ? -zG_x10000 : zG_x10000) / 10000,
         (long)(zG_x10000 < 0 ? -zG_x10000 : zG_x10000) % 10000);

  printf("[MMC5983MA] TESLA: X=%s%lld.%09lld, Y=%s%lld.%09lld, Z=%s%lld.%09lld T\n",
         xnT < 0 ? "-" : "",
         (long long)(xnT < 0 ? -xnT : xnT) / 1000000000LL,
         (long long)(xnT < 0 ? -xnT : xnT) % 1000000000LL,
         ynT < 0 ? "-" : "",
         (long long)(ynT < 0 ? -ynT : ynT) / 1000000000LL,
         (long long)(ynT < 0 ? -ynT : ynT) % 1000000000LL,
         znT < 0 ? "-" : "",
         (long long)(znT < 0 ? -znT : znT) / 1000000000LL,
         (long long)(znT < 0 ? -znT : znT) % 1000000000LL);

  return OK;
}