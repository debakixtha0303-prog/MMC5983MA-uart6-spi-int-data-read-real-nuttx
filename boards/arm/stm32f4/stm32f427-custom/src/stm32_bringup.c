/****************************************************************************
 * boards/arm/stm32f4/stm32f427-custom/src/stm32_bringup.c
 *
 * STM32F427 Custom Board Bringup
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/compiler.h>
#include <nuttx/kthread.h>
#include <nuttx/spi/spi.h>
#include <nuttx/wdog.h>

#include <nuttx/sensors/mmc5983ma.h>

#include "stm32_spi.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct wdog_s g_timer_wdog;

static volatile uint32_t g_timer_count = 0;

/****************************************************************************
 * Timer Callback
 ****************************************************************************/

static void timer_callback(wdparm_t arg)
{
  g_timer_count++;

  wd_start(&g_timer_wdog,
           MSEC2TICK(100),
           timer_callback,
           0);
}

/****************************************************************************
 * Timer Test Thread
 ****************************************************************************/

static int timer_test_thread(int argc, char *argv[])
{
  uint32_t last_count = 0;

  syslog(LOG_INFO,
         "[TIMER] Test thread started\n");

  while (1)
    {
      sleep(1);

      if (g_timer_count != last_count)
        {
          syslog(LOG_INFO,
                 "[TIMER] Trigger count = %lu\n",
                 (unsigned long)g_timer_count);

          last_count = g_timer_count;
        }
    }

  return 0;
}

/****************************************************************************
 * MMC5983MA Reader Thread
 ****************************************************************************/

static int mmc5983ma_reader(int argc, char *argv[])
{
  FAR struct spi_dev_s *spi;
  mmc5983ma_dev_t dev;
  mmc5983ma_data_t data;
  int ret;

  memset(&dev, 0, sizeof(dev));
  memset(&data, 0, sizeof(data));

  syslog(LOG_INFO,
         "[MMC5983MA] Reader starting...\n");

  /**************************************************************************
   * Initialize SPI2
   **************************************************************************/

  spi = stm32_spibus_initialize(2);

  if (spi == NULL)
    {
      syslog(LOG_ERR,
             "[MMC5983MA] SPI2 initialization FAILED\n");

      return -ENODEV;
    }

  syslog(LOG_INFO,
         "[MMC5983MA] SPI2 initialized\n");

  /**************************************************************************
   * Initialize MMC5983MA
   **************************************************************************/

  ret = mmc5983ma_init(&dev,
                        spi,
                        SPIDEV_IMU(0));

  if (ret < 0)
    {
      syslog(LOG_ERR,
             "[MMC5983MA] INIT FAILED: %d\n",
             ret);

      return ret;
    }

  syslog(LOG_INFO,
         "[MMC5983MA] SENSOR READY\n");

  /**************************************************************************
   * Continuous MMC5983MA reading
   **************************************************************************/

  while (1)
    {
      ret = mmc5983ma_read_mag(&dev,
                               &data);

      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "[MMC5983MA] READ FAILED: %d\n",
                 ret);
        }

      /*
       * 100 ms between measurements
       */

      usleep(100000);
    }

  return 0;
}

/****************************************************************************
 * Public: Board Bringup
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret;

  syslog(LOG_INFO,
         "[BRINGUP] Starting...\n");

  /**************************************************************************
   * Start 100 ms Timer
   **************************************************************************/

  ret = wd_start(&g_timer_wdog,
                 MSEC2TICK(100),
                 timer_callback,
                 0);

  if (ret < 0)
    {
      syslog(LOG_ERR,
             "[BRINGUP] Timer start failed: %d\n",
             ret);

      return ret;
    }

  syslog(LOG_INFO,
         "[BRINGUP] Timer started: 100 ms\n");

  /**************************************************************************
   * Start Timer Test Thread
   **************************************************************************/

  ret = kthread_create("timer_test",
                       100,
                       2048,
                       timer_test_thread,
                       NULL);

  if (ret < 0)
    {
      syslog(LOG_ERR,
             "[BRINGUP] Timer test thread failed: %d\n",
             ret);

      wd_cancel(&g_timer_wdog);

      return ret;
    }

  syslog(LOG_INFO,
         "[BRINGUP] Timer test thread started\n");

  /**************************************************************************
   * Initialize SPI2
   **************************************************************************/

  ret = stm32_spibus_initialize(2) != NULL ? 0 : -ENODEV;

  if (ret < 0)
    {
      syslog(LOG_ERR,
             "[BRINGUP] SPI2 init failed\n");

      return ret;
    }

  syslog(LOG_INFO,
         "[BRINGUP] SPI2 OK\n");

  /**************************************************************************
   * Start MMC5983MA Reader Thread
   **************************************************************************/

  ret = kthread_create("mmc5983ma",
                       100,
                       2048,
                       mmc5983ma_reader,
                       NULL);

  if (ret < 0)
    {
      syslog(LOG_ERR,
             "[BRINGUP] MMC thread failed: %d\n",
             ret);

      return ret;
    }

  syslog(LOG_INFO,
         "[BRINGUP] MMC5983MA thread started\n");

  return 0;
}