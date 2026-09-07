#ifndef __INCLUDE_NUTTX_SENSORS_IAM20380_H
#define __INCLUDE_NUTTX_SENSORS_IAM20380_H

#include <nuttx/config.h>
#include <stdint.h>

#define IAM20380_SMPLRT_DIV        0x19
#define IAM20380_CONFIG            0x1A
#define IAM20380_GYRO_CONFIG       0x1B
#define IAM20380_GYRO_XOUT_H       0x43
#define IAM20380_USER_CTRL         0x6A
#define IAM20380_PWR_MGMT_1        0x6B
#define IAM20380_WHO_AM_I          0x75

#define IAM20380_READ_BIT          0x80
#define IAM20380_WHO_AM_I_VAL      0xB5

struct iam20380_rates_s
{
  float gx;
  float gy;
  float gz;
};

struct iam20380_raw_s
{
  int16_t gx;
  int16_t gz;
  int16_t gy;
};

#endif /* __INCLUDE_NUTTX_SENSORS_IAM20380_H */
