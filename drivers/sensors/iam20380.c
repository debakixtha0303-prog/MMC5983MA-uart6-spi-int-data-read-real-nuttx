/****************************************************************************
 * drivers/sensors/iam20380.c
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/signal.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/iam20380.h>
/****************************************************************************
 * Private Types
 ****************************************************************************/

struct iam20380_dev_s
{
  FAR struct spi_dev_s *spi; /* SPI bus interface */
  int spidevid;              /* Chip Select ID */
  float gscale;              /* Scale factor (LSB per deg/s) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void iam20380_configspi(FAR struct spi_dev_s *spi);
static uint8_t iam20380_readreg(FAR struct iam20380_dev_s *priv, uint8_t reg);
static void iam20380_writereg(FAR struct iam20380_dev_s *priv, uint8_t reg, uint8_t val);
static void iam20380_readgyroburst(FAR struct iam20380_dev_s *priv, FAR uint8_t *rx);

/* POSIX File Operations */
static int iam20380_open(FAR struct file *filep);
static int iam20380_close(FAR struct file *filep);
static ssize_t iam20380_read(FAR struct file *filep, FAR char *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_iam20380_fops =
{
  iam20380_open,  /* open */
  iam20380_close, /* close */
  iam20380_read,  /* read */
  0,              /* write */
  0,              /* seek */
  0,              /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void iam20380_configspi(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, 1000000); /* 1 MHz */
}

static uint8_t iam20380_readreg(FAR struct iam20380_dev_s *priv, uint8_t reg)
{
  uint8_t tx[2];
  uint8_t rx[2];

  tx[0] = reg | IAM20380_READ_BIT;
  tx[1] = 0x00;

  iam20380_configspi(priv->spi);
  SPI_SELECT(priv->spi, priv->spidevid, true);
  SPI_EXCHANGE(priv->spi, tx, rx, 2);
  SPI_SELECT(priv->spi, priv->spidevid, false);
  SPI_LOCK(priv->spi, false);

  return rx[1];
}

static void iam20380_writereg(FAR struct iam20380_dev_s *priv, uint8_t reg, uint8_t val)
{
  uint8_t tx[2];
  uint8_t rx[2];

  tx[0] = reg;
  tx[1] = val;

  iam20380_configspi(priv->spi);
  SPI_SELECT(priv->spi, priv->spidevid, true);
  SPI_EXCHANGE(priv->spi, tx, rx, 2);
  SPI_SELECT(priv->spi, priv->spidevid, false);
  SPI_LOCK(priv->spi, false);
}

static void iam20380_readgyroburst(FAR struct iam20380_dev_s *priv, FAR uint8_t *rx)
{
  uint8_t tx[8];
  memset(tx, 0x00, sizeof(tx));
  tx[0] = IAM20380_GYRO_XOUT_H | IAM20380_READ_BIT;

  iam20380_configspi(priv->spi);
  SPI_SELECT(priv->spi, priv->spidevid, true);
  SPI_EXCHANGE(priv->spi, tx, rx, 8);
  SPI_SELECT(priv->spi, priv->spidevid, false);
  SPI_LOCK(priv->spi, false);
}

static int iam20380_open(FAR struct file *filep)
{
  return OK;
}

static int iam20380_close(FAR struct file *filep)
{
  return OK;
}

static ssize_t iam20380_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct iam20380_dev_s *priv = inode->i_private;
  uint8_t rx[8];

  if (buflen < sizeof(struct iam20380_rates_s))
    {
      return -EINVAL;
    }

  iam20380_readgyroburst(priv, rx);

  /* Map registers: rx[2..3]=X, rx[4..5]=Y, rx[6..7]=Z */
  int16_t raw_x = (int16_t)(((uint16_t)rx[2] << 8) | rx[3]);
  int16_t raw_y = (int16_t)(((uint16_t)rx[4] << 8) | rx[5]);
  int16_t raw_z = (int16_t)(((uint16_t)rx[6] << 8) | rx[7]);

  /* Return structured float rate values (dps) */
  FAR struct iam20380_rates_s *data = (FAR struct iam20380_rates_s *)buffer;
  data->gx = (float)raw_x / priv->gscale;
  data->gy = (float)raw_y / priv->gscale;
  data->gz = (float)raw_z / priv->gscale;

  return sizeof(struct iam20380_rates_s);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int iam20380_register(FAR const char *devpath, FAR struct spi_dev_s *spi, int spidevid)
{
  FAR struct iam20380_dev_s *priv;
  uint8_t whoami;
  int ret;

  priv = (FAR struct iam20380_dev_s *)kmm_zalloc(sizeof(struct iam20380_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate IAM20380 memory\n");
      return -ENOMEM;
    }

  priv->spi = spi;
  priv->spidevid = spidevid;
  priv->gscale = 65.5f; /* 500dps scale factor default */

  /* Reset sensor */
  iam20380_writereg(priv, IAM20380_PWR_MGMT_1, 0x81);
  nxsig_usleep(100000);

  /* Disable I2C interface */
  iam20380_writereg(priv, IAM20380_USER_CTRL, 0x10);
  nxsig_usleep(10000);

  /* Read WHO_AM_I */
  whoami = iam20380_readreg(priv, IAM20380_WHO_AM_I);
  sinfo("IAM20380 WHO_AM_I = 0x%02X\n", whoami);

  if (whoami != IAM20380_WHO_AM_I_VAL)
    {
      snerr("ERROR: IAM20380 WHO_AM_I mismatch! Got 0x%02X expected 0x%02X\n",
            whoami, IAM20380_WHO_AM_I_VAL);
      kmm_free(priv);
      return -ENODEV;
    }

  /* Wake up, select Auto Clock (PLL) */
  iam20380_writereg(priv, IAM20380_PWR_MGMT_1, 0x01);

  /* Set DLPF = 20Hz, FSR = 500dps, Sample Rate = 100Hz */
  iam20380_writereg(priv, IAM20380_CONFIG, 0x04);
  iam20380_writereg(priv, IAM20380_GYRO_CONFIG, 0x08); /* 500 dps */
  iam20380_writereg(priv, IAM20380_SMPLRT_DIV, 0x09);

  /* Register POSIX character driver */
  ret = register_driver(devpath, &g_iam20380_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver %s: %d\n", devpath, ret);
      kmm_free(priv);
      return ret;
    }

  sinfo("IAM20380 registered successfully as %s\n", devpath);
  return OK;
}