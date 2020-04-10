/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */
#ifndef H_BSP_DEFS_H
#define H_BSP_DEFS_H

#include "stm32l1xx_hal_dma.h"
#include "stm32l1xx_hal_adc.h"

#ifdef __cplusplus
extern "C" {
#endif


// Set os idle stack size here as we have callups from the os_tick_idle()
#ifdef OS_IDLE_STACK_SIZE
#undef OS_IDLE_STACK_SIZE
#endif
#define OS_IDLE_STACK_SIZE (128)

// The board specific definitions in here for BSP supplied code.


/* UART */
// mynewt device name to access uarts. Note name is mynewt, STM32 doc pin labelling has UART1/2/3
#define UART0_DEV "uart0"
#define UART1_DEV "uart1"
#define UART2_DEV "uart2"
#define UARTDBG_DEV "uartbb0"       // named by uart bitbang code en dur

#define I2C0_DEV "i2c0"         // mynewt bus device name for the I2C1 bus (STM32 numbering)
#define ALTI_NODE "alt0"        // mynewt bus node name for the altimeter
#define ACC_NODE "acc0"          // mynewt bus node name for the accelero

// Mappings for generic code modules are in the syscfg

/* GPIO  type pins */
#define LED_1           MCU_GPIO_PORTA(0)       // also ADC input 0
#define LED_2           MCU_GPIO_PORTA(3)      // No ADC
#define LED_BLINK_PIN   LED_1


// Maker connector
#define CN4_1           MCU_GPIO_PORTB(15)
#define CN4_2           MCU_GPIO_PORTA(12)
#define CN4_3           MCU_GPIO_PORTB(14)
#define CN4_4           MCU_GPIO_PORTA(11)
#define CN4_5           MCU_GPIO_PORTB(13)
#define CN4_6           -1                      // UART RX - DNU
#define CN4_7           MCU_GPIO_PORTB(12)
#define CN4_8           -1                      // UART TX - DNU
#define CN4_9           -1                      // GND
#define CN4_10          -1                      // I2C1 SCL - DNU
#define CN4_11          MCU_GPIO_PORTA(1)
#define CN4_12          -1                      // I2C SDA - DNU

#define SENSOR_PWR      MCU_GPIO_PORTB(6)       // microphone power supply
#define IRQ_ACCEL       MCU_GPIO_PORTA(8)       // irq from accelero

// Map to 'default' function defines for ext io functions if used
#define EXT_LED         (CN4_1)
#define EXT_RELAY1      (CN4_2)
#define EXT_IO          (CN4_3)
#define EXT_RELAY2      (CN4_4)
#define EXT_BUTTON      (CN4_7)
#define BUTTON          (EXT_BUTTON)
// Speaker output PWM drives mosfet for higher power output on speaker connecter also. If no speaker connected, can use as GPIO
#define SPEAKER         (CN4_11)
#define EXT_UART_PWR    -1
#define EXT_I2C_PWR     -1
#define LIGHT_SENSOR    -1
#define LIGHT_SENSOR_ADCCHAN (-1)

/* JTAG on PB3/PA15/PA14/PA13 */

#define BATTERY_GPIO    (128)                   // needs a gpio pin number, but of a value that is beyond 'real' ones
#define BATTERY_ADCCHAN (ADC_CHANNEL_VREFINT)

// ALtimetre I2C chan/addr
// W_PROTO altimeter is a Freescale MPL3115A2
#define ALTI_MPL3115A2 1
#define ALTIMETER_I2C_CHAN 0
#define ALTIMETER_I2C_ADDR 0x5D
// accelerometer I2C chan/addr
// W_PROTO acceleromter is a Freescale MMA7660FCR1 
#define ACC_MMA7660FCR1 1
#define ACCELERO_I2C_CHAN  0
// Default I2C addres for accelero is 0x18, our board pulls SA0 (which is the LSB) high -> 0x19
#define ACCELERO_I2C_ADDR  0x19

//#define Microphone IIS config?
#define I2C_0_FREQUENCY (100000)
#define SPI_0_IRQ_PRIO (2)
#define SPI_0_BAUDRATE (3000)

#ifdef __cplusplus
}
#endif

#endif  /* H_BSP_DEFS_H */
