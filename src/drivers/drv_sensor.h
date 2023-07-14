/****************************************************************************
 *
 *   Copyright (c) 2012-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file drv_sensor.h
 *
 * Common sensor API and ioctl definitions.
 */

#ifndef _DRV_SENSOR_H
#define _DRV_SENSOR_H

#include <px4_platform_common/defines.h>
#include <stdint.h>
#include <sys/ioctl.h>

/**
 * Sensor type definitions.
 *
 * Used to create a unique device id for redundant and combo sensors
 */

#define DRV_MAG_DEVTYPE_HMC5883  0x01

#define DRV_MAG_DEVTYPE_MAGSIM   0x03
#define DRV_MAG_DEVTYPE_AK8963   0x04
#define DRV_MAG_DEVTYPE_LIS3MDL  0x05
#define DRV_MAG_DEVTYPE_IST8310  0x06
#define DRV_MAG_DEVTYPE_RM3100   0x07
#define DRV_MAG_DEVTYPE_QMC5883L 0x08
#define DRV_MAG_DEVTYPE_AK09916  0x09
#define DRV_MAG_DEVTYPE_VCM1193L 0x0A

#define DRV_MAG_DEVTYPE_IST8308  0x0B
#define DRV_MAG_DEVTYPE_LIS2MDL  0x0C

#define DRV_IMU_DEVTYPE_LSM303D  0x11

#define DRV_IMU_DEVTYPE_SIM 0x14
#define DRV_DIFF_PRESS_DEVTYPE_SIM 0x15
#define DRV_FLOW_DEVTYPE_SIM 0x16

#define DRV_IMU_DEVTYPE_MPU6000  0x21
#define DRV_GYR_DEVTYPE_L3GD20   0x22
#define DRV_IMU_DEVTYPE_MPU9250  0x24
#define DRV_IMU_DEVTYPE_ICM20649 0x25
#define DRV_IMU_DEVTYPE_ICM42688P 0x26
#define DRV_IMU_DEVTYPE_ICM40609D 0x27
#define DRV_IMU_DEVTYPE_ICM20948 0x28
#define DRV_IMU_DEVTYPE_ICM42605 0x29
#define DRV_IMU_DEVTYPE_ICM42670P 0x2A
#define DRV_IMU_DEVTYPE_IIM42652 0x2B
#define DRV_IMU_DEVTYPE_IAM20680HP 0x2C
#define DRV_IMU_DEVTYPE_ICM42686P 0x2D

#define DRV_RNG_DEVTYPE_MB12XX   0x31
#define DRV_RNG_DEVTYPE_LL40LS   0x32
#define DRV_ACC_DEVTYPE_MPU6050  0x33
#define DRV_IMU_DEVTYPE_ICM45686 0x34

#define DRV_GYR_DEVTYPE_MPU6050  0x35
#define DRV_IMU_DEVTYPE_MPU6500  0x36
#define DRV_IMU_DEVTYPE_BMI270   0x37
#define DRV_IMU_DEVTYPE_ICM20602 0x38

#define DRV_IMU_DEVTYPE_ICM20608G 0x3A

#define DRV_IMU_DEVTYPE_ICM20689 0x3C
#define DRV_BARO_DEVTYPE_MS5611		0x3D
#define DRV_BARO_DEVTYPE_MS5607		0x3E
#define DRV_BARO_DEVTYPE_BMP280		0x3F
#define DRV_BARO_DEVTYPE_LPS25H		0x40
#define DRV_ACC_DEVTYPE_BMI055		0x41
#define DRV_GYR_DEVTYPE_BMI055		0x42
#define DRV_MAG_DEVTYPE_BMM150		0x43
#define DRV_IMU_DEVTYPE_ST_LSM9DS1_AG   0x44
#define DRV_MAG_DEVTYPE_ST_LSM9DS1_M    0x45

#define DRV_DIFF_PRESS_DEVTYPE_ETS3      0x46
#define DRV_DIFF_PRESS_DEVTYPE_MS4515    0x47
#define DRV_DIFF_PRESS_DEVTYPE_MS4525DO  0x48
#define DRV_DIFF_PRESS_DEVTYPE_MS5525DSO 0x49
#define DRV_DIFF_PRESS_DEVTYPE_SDP31     0x4A
#define DRV_DIFF_PRESS_DEVTYPE_SDP32     0x4B
#define DRV_DIFF_PRESS_DEVTYPE_SDP33     0x4C

#define DRV_BARO_DEVTYPE_TCBP001TA      0x4D
#define DRV_BARO_DEVTYPE_MS5837         0x4E
#define DRV_BARO_DEVTYPE_SPL06          0x4F

#define DRV_AIRDATA_DEVTYPE_NSL_ADU	0x4F

#define DRV_BARO_DEVTYPE_LPS33HW        0x50
#define DRV_BARO_DEVTYPE_MPL3115A2	0x51
#define DRV_ACC_DEVTYPE_FXOS8701C	0x52

#define DRV_GYR_DEVTYPE_FXAS2100C	0x54

#define DRV_IMU_DEVTYPE_ADIS16448	0x57
#define DRV_IMU_DEVTYPE_ADIS16470	0x58
#define DRV_IMU_DEVTYPE_ADIS16477	0x59
#define DRV_IMU_DEVTYPE_ADIS16507	0x5A

#define DRV_BARO_DEVTYPE_MPC2520	0x5F
#define DRV_BARO_DEVTYPE_LPS22HB	0x60

#define DRV_ACC_DEVTYPE_LSM303AGR       0x61
#define DRV_MAG_DEVTYPE_LSM303AGR       0x62
#define DRV_IMU_DEVTYPE_ADIS16497       0x63
#define DRV_BARO_DEVTYPE_BAROSIM        0x65
#define DRV_GYR_DEVTYPE_BMI088          0x66
#define DRV_BARO_DEVTYPE_BMP388         0x67
#define DRV_BARO_DEVTYPE_DPS310         0x68
#define DRV_PWM_DEVTYPE_PCA9685         0x69
#define DRV_ACC_DEVTYPE_BMI088          0x6a
#define DRV_OSD_DEVTYPE_ATXXXX          0x6b
#define DRV_ACC_DEVTYPE_BMI085          0x6C
#define DRV_GYR_DEVTYPE_BMI085          0x6D
#define DRV_BARO_DEVTYPE_BMP390         0x6E

#define DRV_DIST_DEVTYPE_LL40LS       0x70
#define DRV_DIST_DEVTYPE_MAPPYDOT     0x71
#define DRV_DIST_DEVTYPE_MB12XX       0x72
#define DRV_DIST_DEVTYPE_LIGHTWARE_LASER 0x73
#define DRV_DIST_DEVTYPE_SRF02        0x74
#define DRV_DIST_DEVTYPE_TERARANGER   0x75
#define DRV_DIST_DEVTYPE_VL53L0X      0x76

#define DRV_LED_DEVTYPE_RGBLED        0x7a
#define DRV_LED_DEVTYPE_RGBLED_NCP5623C 0x7b
#define DRV_LED_DEVTYPE_RGBLED_IS31FL3195    0xbf
#define DRV_LED_DEVTYPE_RGBLED_LP5562    0xc0

#define DRV_BAT_DEVTYPE_SMBUS         0x7c
#define DRV_SENS_DEVTYPE_IRLOCK       0x7d
#define DRV_SENS_DEVTYPE_PCF8583      0x7e
#define DRV_TEL_DEVTYPE_BST           0x7f

// Generic types for unknown CAN sensors
#define DRV_ACC_DEVTYPE_UAVCAN	0x80
#define DRV_BARO_DEVTYPE_UAVCAN	0x81
#define DRV_BAT_DEVTYPE_UAVCAN	0x82
#define DRV_DIFF_PRESS_DEVTYPE_UAVCAN	0x83
#define DRV_FLOW_DEVTYPE_UAVCAN	0x84
#define DRV_GPS_DEVTYPE_UAVCAN	0x85
#define DRV_GYR_DEVTYPE_UAVCAN	0x86
#define DRV_IMU_DEVTYPE_UAVCAN	0x87
#define DRV_MAG_DEVTYPE_UAVCAN	0x88
#define DRV_DIST_DEVTYPE_UAVCAN	0x89

#define DRV_ADC_DEVTYPE_ADS1115        0x90

#define DRV_DIST_DEVTYPE_VL53L1X       0x91
#define DRV_DIST_DEVTYPE_CM8JL65       0x92
#define DRV_DIST_DEVTYPE_LEDDARONE     0x93
#define DRV_DIST_DEVTYPE_MAVLINK       0x94
#define DRV_DIST_DEVTYPE_PGA460        0x95
#define DRV_DIST_DEVTYPE_PX4FLOW       0x96
#define DRV_DIST_DEVTYPE_TFMINI        0x97
#define DRV_DIST_DEVTYPE_ULANDING      0x98
#define DRV_DIST_DEVTYPE_AFBRS50       0x99
#define DRV_DIST_DEVTYPE_SIM           0x9A
#define DRV_DIST_DEVTYPE_SRF05         0x9B
#define DRV_DIST_DEVTYPE_GY_US42       0x9C

#define DRV_BAT_DEVTYPE_BATMON_SMBUS   0x9d
#define DRV_GPIO_DEVTYPE_MCP23009      0x9F

#define DRV_GPS_DEVTYPE_ASHTECH 0xA0
#define DRV_GPS_DEVTYPE_EMLID_REACH 0xA1
#define DRV_GPS_DEVTYPE_FEMTOMES 0xA2
#define DRV_GPS_DEVTYPE_MTK 0xA3
#define DRV_GPS_DEVTYPE_SBF 0xA4
#define DRV_GPS_DEVTYPE_UBX     0xA5
#define DRV_GPS_DEVTYPE_UBX_6   0xA6
#define DRV_GPS_DEVTYPE_UBX_7   0xA7
#define DRV_GPS_DEVTYPE_UBX_8   0xA8
#define DRV_GPS_DEVTYPE_UBX_9   0xA9
#define DRV_GPS_DEVTYPE_UBX_F9P 0xAA
#define DRV_GPS_DEVTYPE_NMEA 0xAB

#define DRV_GPS_DEVTYPE_SIM 0xAF

#define DRV_TRNS_DEVTYPE_MXS 0xB0

#define DRV_HYGRO_DEVTYPE_SHT3X 0xB1

#define DRV_FLOW_DEVTYPE_MAVLINK 0xB2
#define DRV_FLOW_DEVTYPE_PMW3901 0xB3
#define DRV_FLOW_DEVTYPE_PAW3902 0xB4
#define DRV_FLOW_DEVTYPE_PX4FLOW 0xB5
#define DRV_FLOW_DEVTYPE_PAA3905 0xB6

#define DRV_BARO_DEVTYPE_ICP101XX 0xB7
#define DRV_BARO_DEVTYPE_ICP201XX 0xB8

#define DRV_POWER_DEVTYPE_INA226 0xD0
#define DRV_POWER_DEVTYPE_INA228 0xD1
#define DRV_POWER_DEVTYPE_VOXLPM 0xD2
#define DRV_POWER_DEVTYPE_INA220 0xD3
#define DRV_POWER_DEVTYPE_INA238 0xD4

#define DRV_DIST_DEVTYPE_TF02PRO 0xE0

#define DRV_INS_DEVTYPE_VN100 0xE1
#define DRV_INS_DEVTYPE_VN200 0xE2
#define DRV_INS_DEVTYPE_VN300 0xE3

#define DRV_DEVTYPE_UNUSED		0xff

#endif /* _DRV_SENSOR_H */
