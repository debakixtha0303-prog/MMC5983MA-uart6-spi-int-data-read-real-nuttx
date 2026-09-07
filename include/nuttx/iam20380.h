/****************************************************************************
 * drivers/sensors/iam20380.h
 ****************************************************************************/

#ifndef __DRIVERS_SENSORS_IAM20380_H
#define __DRIVERS_SENSORS_IAM20380_H

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <stdint.h>

struct iam20380_dev_s;

/* Allocate, initialize (runs the full IAM_begin-equivalent sequence),
 * and return a handle. Returns NULL if WHO_AM_I doesn't match.
 *
 * spi      - already-initialized SPI bus (e.g. from stm32_spibus_initialize)
 * spidevid - chip select identifier used with SPI_SELECT (board-specific)
 */

FAR struct iam20380_dev_s *iam20380_register(FAR struct spi_dev_s *spi,
                                              int spidevid);

/* Read the WHO_AM_I register directly (0x70 expected on this hardware) */

uint8_t iam20380_whoami(FAR struct iam20380_dev_s *priv);

/* Run the init sequence again if needed */

int iam20380_init(FAR struct iam20380_dev_s *priv);

/* Burst-read gyro X/Y/Z as raw int16 (divide by scale factor for deg/s) */

void iam20380_getgyro(FAR struct iam20380_dev_s *priv,
                      FAR int16_t *gx, FAR int16_t *gy, FAR int16_t *gz);

#endif /* __DRIVERS_SENSORS_IAM20380_H */
