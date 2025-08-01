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

#define FC_TARGET_MCU     STM32F7X2

#define BOARD_NAME        HOBBYWING_XROTORF7CONV
#define MANUFACTURER_ID   HOWI

#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_ACC_SPI_ICM42688P
#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_GYRO_SPI_ICM42688P
#define USE_BARO_DPS310
#define USE_BARO_BMP280
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

#define BEEPER_PIN           PC14
#define MOTOR1_PIN           PB1
#define MOTOR2_PIN           PB4
#define MOTOR3_PIN           PB3
#define MOTOR4_PIN           PA15
#define MOTOR5_PIN           PC8
#define MOTOR6_PIN           PC9
#define RX_PPM_PIN           PC6
#define RX_PWM1_PIN          PB10
#define RX_PWM2_PIN          PA2
#define RX_PWM3_PIN          PA3
#define LED_STRIP_PIN        PA8
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PA2
#define UART3_TX_PIN         PB10
#define UART4_TX_PIN         PA0
#define UART5_TX_PIN         PC12
#define UART6_TX_PIN         PC6
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PA3
#define UART3_RX_PIN         PB11
#define UART4_RX_PIN         PA1
#define UART5_RX_PIN         PD2
#define UART6_RX_PIN         PC7
#define I2C1_SCL_PIN         PB6
#define I2C1_SDA_PIN         PB7
#define LED0_PIN             PC15
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI3_SCK_PIN         PC10
#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PB14
#define SPI3_SDI_PIN         PC11
#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PB15
#define SPI3_SDO_PIN         PB5
#define CAMERA_CONTROL_PIN   PB8
#define ADC_VBAT_PIN         PC1
#define ADC_RSSI_PIN         PC2
#define ADC_CURR_PIN         PC0
#define PINIO1_PIN           PB0
#define PINIO2_PIN           PB9
#define FLASH_CS_PIN         PC13
#define MAX7456_SPI_CS_PIN   PB12
#define GYRO_1_EXTI_PIN      PC3
#define GYRO_2_EXTI_PIN      PC4
#define GYRO_1_CS_PIN        PA4
#define GYRO_2_CS_PIN        PB2

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PC6 , 2,  0) \
    TIMER_PIN_MAP( 1, PB1 , 2,  0) \
    TIMER_PIN_MAP( 2, PB4 , 1,  0) \
    TIMER_PIN_MAP( 3, PB3 , 1,  0) \
    TIMER_PIN_MAP( 4, PA15, 1,  0) \
    TIMER_PIN_MAP( 5, PC8 , 2,  1) \
    TIMER_PIN_MAP( 6, PC9 , 2,  0) \
    TIMER_PIN_MAP( 7, PA8 , 1,  0) \
    TIMER_PIN_MAP( 8, PB8 , 2, -1) \
    TIMER_PIN_MAP( 9, PB10, 1,  0) \
    TIMER_PIN_MAP(10, PA2 , 2,  0) \
    TIMER_PIN_MAP(11, PA3 , 2,  1)


#define ADC3_DMA_OPT        0

#define MAG_I2C_INSTANCE I2CDEV_1
#define USE_BARO
#define BARO_I2C_INSTANCE I2CDEV_1
#define ADC_INSTANCE ADC3
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_FLASH
#define DEFAULT_DSHOT_BURST DSHOT_DMAR_ON
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE 170
#define BEEPER_INVERTED
#define MAX7456_SPI_INSTANCE SPI2
#define FLASH_SPI_INSTANCE SPI3
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW90_DEG
#define GYRO_2_SPI_INSTANCE SPI1
#define GYRO_2_ALIGN CW270_DEG
