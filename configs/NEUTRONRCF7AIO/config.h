/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define FC_TARGET_MCU     STM32F745

#define BOARD_NAME        NEUTRONRCF7AIO
#define MANUFACTURER_ID   NERC

#define USE_ACC
#define USE_ACC_SPI_ICM20689
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_ICM42688P
#define USE_GYRO
#define USE_GYRO_SPI_ICM20689
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_ICM42688P
#define USE_ACCGYRO_BMI270
#define USE_BARO_DPS310
#define USE_FLASH
#define USE_FLASH_W25N01G
#define USE_FLASH_M25P16
#define USE_MAX7456

#define BEEPER_PIN           PD2
#define MOTOR1_PIN           PB0
#define MOTOR2_PIN           PB1
#define MOTOR3_PIN           PB4
#define MOTOR4_PIN           PB5
#define MOTOR5_PIN           PD12
#define MOTOR6_PIN           PD13
#define MOTOR7_PIN           PC8
#define MOTOR8_PIN           PC9
#define LED_STRIP_PIN        PA8
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PA2
#define UART3_TX_PIN         PB10
#define UART4_TX_PIN         PA0
#define UART6_TX_PIN         PC6
#define UART7_TX_PIN         PE8
#define UART8_TX_PIN         PE1
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PA3
#define UART3_RX_PIN         PB11
#define UART4_RX_PIN         PA1
#define UART6_RX_PIN         PC7
#define UART7_RX_PIN         PE7
#define UART8_RX_PIN         PE0
#define I2C1_SCL_PIN         PB8
#define I2C2_SCL_PIN         PB10
#define I2C1_SDA_PIN         PB9
#define I2C2_SDA_PIN         PB11
#define LED0_PIN             PC13
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI3_SCK_PIN         PC10
#define SPI4_SCK_PIN         PE2
#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PB14
#define SPI3_SDI_PIN         PC11
#define SPI4_SDI_PIN         PE5
#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PB15
#define SPI3_SDO_PIN         PC12
#define SPI4_SDO_PIN         PE6
#define CAMERA_CONTROL_PIN   PB3
#define ADC_VBAT_PIN         PC3
#define ADC_RSSI_PIN         PC5
#define ADC_CURR_PIN         PC2
#define ADC_EXTERNAL1_PIN    PC1
#define FLASH_CS_PIN         PA15
#define MAX7456_SPI_CS_PIN   PE4
#define GYRO_1_EXTI_PIN      PD0
#define GYRO_2_EXTI_PIN      PD8
#define GYRO_1_CS_PIN        PA4
#define GYRO_2_CS_PIN        PB12

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PA8 , 1,  0) \
    TIMER_PIN_MAP( 1, PB3 , 1,  0) \
    TIMER_PIN_MAP( 2, PB0 , 2,  0) \
    TIMER_PIN_MAP( 3, PB1 , 2,  0) \
    TIMER_PIN_MAP( 4, PB4 , 1,  0) \
    TIMER_PIN_MAP( 5, PB5 , 1,  0) \
    TIMER_PIN_MAP( 6, PD12, 1,  0) \
    TIMER_PIN_MAP( 7, PD13, 1,  0) \
    TIMER_PIN_MAP( 8, PC8 , 2,  1) \
    TIMER_PIN_MAP( 9, PC9 , 2,  0)


#define ADC1_DMA_OPT        0

#define MAG_ALIGN CW180_DEG
#define MAG_ALIGN_YAW 1800
#define MAG_I2C_INSTANCE I2CDEV_1
#define USE_BARO
#define BARO_I2C_INSTANCE I2CDEV_1

#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_FLASH
#define DEFAULT_DSHOT_TELEMETRY DSHOT_TELEMETRY_ON
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_VOLTAGE_METER_SCALE 210
#define DEFAULT_CURRENT_METER_SCALE 100
#define BEEPER_INVERTED
#define MAX7456_SPI_INSTANCE SPI4
#define FLASH_SPI_INSTANCE SPI3
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_2_SPI_INSTANCE SPI2
#define GYRO_2_ALIGN CW180_DEG
