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

#define FC_TARGET_MCU     STM32F405

#define BOARD_NAME        RATEF405_V3
#define MANUFACTURER_ID   RATE

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_FLASH
#define USE_FLASH_W25Q128FV

#define BEEPER_PIN           PB11
#define MOTOR1_PIN           PC09
#define MOTOR2_PIN           PA08
#define MOTOR3_PIN           PA09
#define MOTOR4_PIN           PA10
#define MOTOR5_PIN           NONE
#define MOTOR6_PIN           NONE
#define MOTOR7_PIN           NONE
#define MOTOR8_PIN           NONE
#define RX_PPM_PIN           PA03
#define LED_STRIP_PIN        PB01
#define UART1_TX_PIN         PB06
#define UART2_TX_PIN         PA02
#define UART3_TX_PIN         PC10
#define UART4_TX_PIN         PA00
#define UART5_TX_PIN         PC12
#define UART6_TX_PIN         PC06
#define UART1_RX_PIN         PB07
#define UART2_RX_PIN         PA03
#define UART3_RX_PIN         PC11
#define UART4_RX_PIN         PA01
#define UART5_RX_PIN         PD02
#define UART6_RX_PIN         PC07
#define I2C1_SCL_PIN         PB08
#define I2C1_SDA_PIN         PB09
#define LED0_PIN             PC04
#define SPI1_SCK_PIN         PA05
#define SPI2_SCK_PIN         PB13
#define SPI3_SCK_PIN         PB03
#define SPI1_SDI_PIN         PA06
#define SPI2_SDI_PIN         PB14
#define SPI3_SDI_PIN         PB04
#define SPI1_SDO_PIN         PA07
#define SPI2_SDO_PIN         PB15
#define SPI3_SDO_PIN         PB05
#define ESCSERIAL_PIN        PC07
#define ADC_VBAT_PIN         PB00
#define ADC_CURR_PIN         PC05
#define FLASH_CS_PIN         PA04
#define GYRO_1_EXTI_PIN      PB10
#define GYRO_1_CS_PIN        PB12

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PA03, 2,  0) \
    TIMER_PIN_MAP( 1, PC09, 3,  0) \
    TIMER_PIN_MAP( 2, PA08, 1,  0) \
    TIMER_PIN_MAP( 3, PA09, 1,  0) \
    TIMER_PIN_MAP( 4, PA10, 1,  0) \
    TIMER_PIN_MAP( 5, PB01, 2,  0)

#define ADC1_DMA_OPT 0

#define MAG_I2C_INSTANCE I2CDEV_1
#define ADC_INSTANCE ADC1
#define DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_FLASH
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define BEEPER_INVERTED
#define FLASH_SPI_INSTANCE SPI1
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW0_DEG
#define SYSTEM_HSE_MHZ 8
