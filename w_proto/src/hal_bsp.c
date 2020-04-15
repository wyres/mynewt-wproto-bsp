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
#include <assert.h>

#include "os/mynewt.h"

#include "bsp/bsp.h"
#include "sysflash/sysflash.h"


#define RAM_SIZE    (32 * 1024)         // TODO this should come from bsp.yml
//#define PROM_BASE sysflash_map_dflt[EEPROM_AREA].fa_off
//#define PROM_SIZE sysflash_map_dflt[EEPROM_AREA].fa_size
#define PROM_BASE (0x08080000)      // MYNEWT_VAL(bsp.eeprom_map.EEPROM_AREA.offset)      ?? how to get from bsp.yml
#define PROM_SIZE   (8*1024)            // TODO this should also come from bsp.yml as .size

#if MYNEWT_VAL(UART_0) || MYNEWT_VAL(UART_DBG)
#include <uart/uart.h>
#endif
#if MYNEWT_VAL(UART_0)
#include <uart_hal/uart_hal.h>
#endif
#if MYNEWT_VAL(UART_DBG)
#include <uart_bitbang/uart_bitbang.h>
#endif

#include <hal/hal_bsp.h>
#include <hal/hal_gpio.h>
#include <hal/hal_flash_int.h>
#include <hal/hal_timer.h>

#if MYNEWT_VAL(ADC) 
//#include <adc/adc.h>
#include "stm32l1xx_hal_adc.h"
#include "stm32l1xx_hal_rcc.h"
#include "stm32l1xx_hal.h"

#endif

#if MYNEWT_VAL(SPI_0_MASTER) || MYNEWT_VAL(SPI_0_SLAVE) || MYNEWT_VAL(SPI_1_MASTER) || MYNEWT_VAL(SPI_1_SLAVE)
#include <hal/hal_spi.h>
#endif
#if MYNEWT_VAL(I2C_0)
#if MYNEWT_VAL(USE_BUS_I2C)
#include "bus/drivers/i2c_common.h"
#include "bus/drivers/i2c_hal.h"
#else
#include "hal/hal_i2c.h"
#endif  /* USE_BUS_I2C */
#endif      /* I2C_0 */

#include <stm32l151xc.h>
#include <stm32l1xx_hal_rcc.h>
#include <stm32l1xx_hal_pwr.h>
#include <stm32l1xx_hal_flash.h>
#include <stm32l1xx_hal_gpio_ex.h>
#include <mcu/stm32l1_bsp.h>
#include "mcu/stm32l1xx_mynewt_hal.h"
#include "mcu/stm32_hal.h"
#if MYNEWT_VAL(RTC)
#include "hal/hal_rtc.h"
#endif

#include "bsp/bsp.h"

extern void hal_mcu_halt();  

/* Uart0 is UART1 in STM32 doc hence names of HAL defns */
#if MYNEWT_VAL(UART_0)
static struct uart_dev hal_uart0;

static const struct stm32_uart_cfg os_bsp_uart0_cfg = {
        .suc_uart = USART1,      
        .suc_rcc_reg = &RCC->APB2ENR,
        .suc_rcc_dev = RCC_APB2ENR_USART1EN,
        .suc_pin_tx = MYNEWT_VAL(UART_0_PIN_TX),
        .suc_pin_rx = MYNEWT_VAL(UART_0_PIN_RX),
        .suc_pin_rts = MYNEWT_VAL(UART_0_PIN_RTS),
        .suc_pin_cts = MYNEWT_VAL(UART_0_PIN_CTS),
        .suc_pin_af = GPIO_AF7_USART1,
        .suc_irqn = USART1_IRQn
};
#endif


/* UartDbg is bitbang on a gpio - initialised by the bitbang package in sysinit */

#if MYNEWT_VAL(ADC) 
/*static struct acd_dev hal_adc_dev;
static const struct stm32_adc_dev_cfg adc_cfg = {
    .c_refmv=0,
    .c_res=12,
    .c_configured=true,
};*/
#endif

#if MYNEWT_VAL(I2C_0)
#if MYNEWT_VAL(USE_BUS_I2C)
static struct bus_i2c_dev bus_i2c0;
static struct bus_i2c_dev_cfg cfg_i2c0 = {
    .i2c_num = 0,
    .pin_sda = MYNEWT_VAL(I2C_0_PIN_SDA),   //I2C_0_SDA,
    .pin_scl = MYNEWT_VAL(I2C_0_PIN_SCL),   //I2C_0_SCL,
};
static struct bus_i2c_node node_alti;
static struct bus_i2c_node_cfg cfg_alti = {
    .node_cfg.bus_name=I2C0_DEV,
    .node_cfg.lock_timeout_ms=0,
    .addr = ALTIMETER_I2C_ADDR,
    .freq = 100,
    .quirks = 0,
};
static struct bus_i2c_node node_acc;
static struct bus_i2c_node_cfg cfg_acc = {
    .node_cfg.bus_name=I2C0_DEV,
    .node_cfg.lock_timeout_ms=0,
    .addr = ACCELERO_I2C_ADDR,
    .freq = 100,
    .quirks = 0,
};
#else   /* USE_BUS_I2C */
// Note: I2C0 is I2C1 in STM32 doc hence names of defns from HAL
struct stm32_hal_i2c_cfg os_bsp_i2c0_cfg = {
    .hic_i2c = I2C1,
    .hic_rcc_reg = &RCC->APB1ENR,
    .hic_rcc_dev = RCC_APB1ENR_I2C1EN,
    .hic_pin_sda = MYNEWT_VAL(I2C_0_PIN_SDA),   // I2C_0_SDA,         // ok
    .hic_pin_scl = MYNEWT_VAL(I2C_0_PIN_SCL),   //I2C_0_SCL,         // ok
    .hic_pin_af = GPIO_AF4_I2C1,
    .hic_10bit = 0,
    .hic_speed = I2C_0_FREQUENCY                     // 100kHz 
};
#endif /* USE_BUS_I2C */

#endif

// SPI0 (mynewt) refers to SPI1 in STM32 doc
#if MYNEWT_VAL(SPI_0_SLAVE) || MYNEWT_VAL(SPI_0_MASTER)
struct stm32_hal_spi_cfg os_bsp_spi0_cfg = {
    .sck_pin = MYNEWT_VAL(SPI_0_PIN_SCK),
    .mosi_pin = MYNEWT_VAL(SPI_0_PIN_MOSI),
    .miso_pin = MYNEWT_VAL(SPI_0_PIN_MISO),
    .ss_pin = MYNEWT_VAL(SPI_0_PIN_SS),
    .irq_prio = SPI_0_IRQ_PRIO,
};
#endif
#if MYNEWT_VAL(SPI_1_SLAVE) || MYNEWT_VAL(SPI_1_MASTER)
struct stm32_hal_spi_cfg os_bsp_spi1_cfg = {
    .sck_pin = MYNEWT_VAL(SPI_1_PIN_SCK),
    .mosi_pin = MYNEWT_VAL(SPI_1_PIN_MOSI),
    .miso_pin = MYNEWT_VAL(SPI_1_PIN_MISO),
    .ss_pin = MYNEWT_VAL(SPI_1_PIN_SS),
    .irq_prio = SPI_1_IRQ_PRIO,
};
#endif

#if MYNEWT_VAL(RTC)
struct stm32_hal_rtc_cfg rtc_cfg = {
    .hrc_hour_fmt = 24,
    .hrc_a_prediv = 31,
    .hrc_s_prediv = 1023
};
#endif


static const struct hal_bsp_mem_dump dump_cfg[] = {
    [0] = {
        .hbmd_start = &_ram_start,
        .hbmd_size = RAM_SIZE
    },
};

extern const struct hal_flash stm32_flash_dev;
const struct hal_flash *
hal_bsp_flash_dev(uint8_t id)
{
    /*
     * Internal flash mapped to id 0.
     */
    if (id != 0) {
        return NULL;
    }
    return &stm32_flash_dev;
}

const struct hal_bsp_mem_dump *
hal_bsp_core_dump(int *area_cnt)
{
    *area_cnt = sizeof(dump_cfg) / sizeof(dump_cfg[0]);
    return dump_cfg;
}

void
hal_bsp_init(void)
{
    int rc;

#if MYNEWT_VAL(UART_0)
    rc = os_dev_create((struct os_dev *) &hal_uart0, UART0_DEV,
      OS_DEV_INIT_PRIMARY, 0, uart_hal_init, (void *)&os_bsp_uart0_cfg);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(TIMER_0)
    hal_timer_init(0, MYNEWT_VAL(TIMER_0_TIM));
#endif

#if MYNEWT_VAL(TIMER_1)    
    hal_timer_init(1, MYNEWT_VAL(TIMER_1_TIM));
#endif

#if MYNEWT_VAL(TIMER_2)
    hal_timer_init(2, MYNEWT_VAL(TIMER_2_TIM));
#endif

#if (MYNEWT_VAL(OS_CPUTIME_TIMER_NUM) >= 0)
    rc = os_cputime_init(MYNEWT_VAL(OS_CPUTIME_FREQ));
    assert(rc == 0);
#endif

#if MYNEWT_VAL(ADC)
/*  device for adc not yet possible as only STM32F4 driver  
    rc = os_dev_create((struct os_dev *) &hal_adc, ADC_DEV,
      OS_DEV_INIT_PRIMARY, 0, adc_hal_init, (void *)&adc_cfg);
    assert(rc == 0);
    */
#endif

    rc = hal_bsp_init_spi();
    assert(rc ==0);

    rc = hal_bsp_init_i2c();
    assert(rc ==0);
    // only init i2c here if using bus driver - else the i2c is init/deinit() on each usage
#if MYNEWT_VAL(USE_BUS_I2C)
    // Create I2C0 bus driver
    rc = bus_i2c_hal_dev_create(I2C0_DEV, &bus_i2c0, &cfg_i2c0);
    assert(rc == 0);
    // And the I2C nodes on the W_BASE card
    // Altimeter
    rc = bus_i2c_node_create(ALTI_NODE, &node_alti,
                    &cfg_alti, &(cfg_alti.node_cfg));
    assert(rc == 0);
    // Accelero
    rc = bus_i2c_node_create(ACC_NODE, &node_acc,
                    &cfg_acc, &(cfg_acc.node_cfg));
    assert(rc == 0);
#endif /* USE_BUS_I2C */

}

/**
 * Returns the configured priority for the given interrupt. If no priority
 * configured, return the priority passed in
 *
 * @param irq_num
 * @param pri
 *
 * @return uint32_t
 */
uint32_t
hal_bsp_get_nvic_priority(int irq_num, uint32_t pri)
{
    /* Add any interrupt priorities configured by the bsp here */
    return pri;
}


int hal_bsp_init_spi(void) {

    int rc=0;

// note : SPI0 is SPI1 in STM32 doc
#if MYNEWT_VAL(SPI_0_MASTER)
    rc = hal_spi_init(0, &os_bsp_spi0_cfg, HAL_SPI_TYPE_MASTER);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(SPI_0_SLAVE)
    rc = hal_spi_init(0, &os_bsp_spi0_cfg, HAL_SPI_TYPE_SLAVE);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(SPI_1_MASTER)
    // MyNewt numbers devices from 0
    rc = hal_spi_init(1, &os_bsp_spi1_cfg, HAL_SPI_TYPE_MASTER);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(SPI_1_SLAVE)
    rc = hal_spi_init(1, &os_bsp_spi1_cfg, HAL_SPI_TYPE_SLAVE);
    assert(rc == 0);
#endif
    return rc;

/*ANOTHER DEINIT METHOD */
/*
#if MYNEWT_VAL(SPI_0_MASTER) || MYNEWT_VAL(SPI_0_SLAVE)
    hal_spi_enable(0);
#endif

#if MYNEWT_VAL(SPI_1_MASTER) || MYNEWT_VAL(SPI_1_SLAVE)
    hal_spi_enable(1);
#endif
*/

}

int hal_bsp_deinit_spi(void){

// note : SPI0 is SPI1 in STM32 doc
#if MYNEWT_VAL(SPI_0_SLAVE) || MYNEWT_VAL(SPI_0_MASTER)
    
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();
    __HAL_RCC_SPI1_CLK_DISABLE( );

    hal_gpio_deinit(os_bsp_spi0_cfg.ss_pin);
    hal_gpio_init_in(os_bsp_spi0_cfg.ss_pin, HAL_GPIO_PULL_UP);

    hal_gpio_deinit(os_bsp_spi0_cfg.sck_pin);
    hal_gpio_init_in(os_bsp_spi0_cfg.sck_pin, HAL_GPIO_PULL_DOWN);
    
    hal_gpio_deinit(os_bsp_spi0_cfg.miso_pin);
    hal_gpio_init_in(os_bsp_spi0_cfg.miso_pin, HAL_GPIO_PULL_DOWN);

    hal_gpio_deinit(os_bsp_spi0_cfg.mosi_pin);
    hal_gpio_init_in(os_bsp_spi0_cfg.mosi_pin, HAL_GPIO_PULL_DOWN);

#endif

#if MYNEWT_VAL(SPI_1_SLAVE) || MYNEWT_VAL(SPI_1_MASTER)
    
    __HAL_RCC_SPI2_FORCE_RESET();
    __HAL_RCC_SPI2_RELEASE_RESET();
    __HAL_RCC_SPI2_CLK_DISABLE( );

    hal_gpio_deinit(os_bsp_spi1_cfg.ss_pin);
    hal_gpio_init_in(os_bsp_spi1_cfg.ss_pin, HAL_GPIO_PULL_UP);

    hal_gpio_deinit(os_bsp_spi1_cfg.sck_pin);
    hal_gpio_init_in(os_bsp_spi1_cfg.sck_pin, HAL_GPIO_PULL_DOWN);
    
    hal_gpio_deinit(os_bsp_spi1_cfg.miso_pin);
    hal_gpio_init_in(os_bsp_spi1_cfg.miso_pin, HAL_GPIO_PULL_DOWN);

    hal_gpio_deinit(os_bsp_spi1_cfg.mosi_pin);
    hal_gpio_init_in(os_bsp_spi1_cfg.mosi_pin, HAL_GPIO_PULL_DOWN);

#endif
    return 0;
}

#if MYNEWT_VAL(USE_BUS_I2C)
// need these as STM32 MCU HAL code does NOT define them, and the I2C mynewt driver code requires them
int hal_i2c_disable(uint8_t n) {
    return 0;
}
int hal_i2c_enable(uint8_t n) {
    return 0;
}
int hal_i2c_init_hw(uint8_t i2c_num, const struct hal_i2c_hw_settings *cfg) {
    return 0;
}
int hal_i2c_config(uint8_t i2c_num, const struct hal_i2c_settings *cfg) {
    
    return 0;
}
#endif /* USE_BUS_I2C */

// initialise I2C
int hal_bsp_init_i2c() {
    int rc = 0;
#if MYNEWT_VAL(I2C_0)
    rc = hal_i2c_init(0, &os_bsp_i2c0_cfg);
#endif
#if MYNEWT_VAL(I2C_1)
    rc = hal_i2c_init(1, &os_bsp_i2c1_cfg);
#endif
#if MYNEWT_VAL(I2C_2)
    rc = hal_i2c_init(2, &os_bsp_i2c2_cfg);
#endif
    return rc;
}
// deinit for power saving
int hal_bsp_deinit_i2c() {
    
#if MYNEWT_VAL(I2C_0)

    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
    __HAL_RCC_I2C1_CLK_DISABLE( );

#if 0
    /*deinit is not done properly because of this : */
    hal_gpio_deinit(os_bsp_i2c0_cfg.hic_pin_sda);
    hal_gpio_init_in(os_bsp_i2c0_cfg.hic_pin_sda, HAL_GPIO_PULL_UP);
    
    hal_gpio_deinit(os_bsp_i2c0_cfg.hic_pin_scl);
   	hal_gpio_init_in(os_bsp_i2c0_cfg.hic_pin_scl, HAL_GPIO_PULL_UP);    
#endif

#endif
    return 0;
}

// NVM access - we have a EEPROM on this MCU which is handy
uint16_t hal_bsp_nvmSize() {
    return PROM_SIZE;
}
bool hal_bsp_nvmLock() {
    return (HAL_FLASHEx_DATAEEPROM_Lock()==HAL_OK);
}
bool hal_bsp_nvmUnlock() {
    return (HAL_FLASHEx_DATAEEPROM_Unlock()==HAL_OK);
}
uint8_t hal_bsp_nvmRead8(uint16_t off) {
    return *((volatile uint8_t*)(PROM_BASE+off));
}
uint16_t hal_bsp_nvmRead16(uint16_t off) {
    return *((volatile uint16_t*)(PROM_BASE+off));
}
bool hal_bsp_nvmRead(uint16_t off, uint8_t len, uint8_t* buf) {
    for(int i=0;i<len;i++) {
        *(buf+i) = hal_bsp_nvmRead8(off+i);
    }
    return true;
}

bool hal_bsp_nvmWrite8(uint16_t off, uint8_t v) {
    HAL_FLASHEx_DATAEEPROM_Erase(FLASH_TYPEERASEDATA_BYTE, ((uint32_t)PROM_BASE)+off);
    return (HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_FASTBYTE, ((uint32_t)PROM_BASE)+off, v)==HAL_OK);
}
bool hal_bsp_nvmWrite16(uint16_t off, uint16_t v) {
    HAL_FLASHEx_DATAEEPROM_Erase(FLASH_TYPEERASEDATA_HALFWORD, ((uint32_t)PROM_BASE)+off);
    return (HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_FASTHALFWORD, ((uint32_t)PROM_BASE)+off, v)==HAL_OK);
}
bool hal_bsp_nvmWrite(uint16_t off, uint8_t len, uint8_t* buf) {
    bool ret = true;
    for(int i=0;i<len;i++) {
        ret &= hal_bsp_nvmWrite8(off+i, *(buf+i));
    }
    return ret;
}

// hwrev value is in build, but can be overridden by app code (eg from a config value at boot time)
static int _hwrev = MYNEWT_VAL(BSP_HW_REV);
int BSP_getHwVer() {
    return _hwrev;
}
void BSP_setHwVer(int v) {
    _hwrev = v;
}
// BSP specific functions to set the radio antenna switch correctly
// For W_BASE card, 2 pins are ALWAYS required (revB or revC or later)
void BSP_antSwInit(int txPin, int rxPin) {
    assert(txPin!=-1);
    assert(rxPin!=-1);
    hal_gpio_init_out(txPin, 0);
    hal_gpio_init_out(rxPin, 0);
}
void BSP_antSwDeInit(int txPin, int rxPin) {
    assert(txPin!=-1);
    assert(rxPin!=-1);
    hal_gpio_init_in(txPin, HAL_GPIO_PULL_NONE);
    hal_gpio_init_in(rxPin, HAL_GPIO_PULL_NONE);
}
void BSP_antSwTx(int txPin, int rxPin) {
    assert(txPin!=-1);
    assert(rxPin!=-1);
    hal_gpio_write(txPin, 1);
    hal_gpio_write(rxPin, 0);
}
void BSP_antSwRx(int txPin, int rxPin) {
    assert(txPin!=-1);
    assert(rxPin!=-1);
    hal_gpio_write(txPin, 0);
    hal_gpio_write(rxPin, 1);
}



#if MYNEWT_VAL(ADC) 
// Initialise an adc for basic gpio like use
static struct {
    ADC_HandleTypeDef adcHandle;
    bool active;
} _adc1 = {
    .adcHandle= {        
        .Instance = ADC1,
    },
    .active=false,
};

bool hal_bsp_adc_init() {
    // allow mulitple calls to init without issues
    if (!_adc1.active) {
        // Configure ADC
        ADC_HandleTypeDef* adch = &_adc1.adcHandle;

#if MYNEWT_VAL(STM32_CLOCK_HSE)
        // Enable HSI - already done in clock setup
        __HAL_RCC_HSI_ENABLE( );

        // Wait till HSI is ready
        while( __HAL_RCC_GET_FLAG( RCC_FLAG_HSIRDY ) == RESET );
#endif
        __HAL_RCC_ADC1_CLK_ENABLE( );
        // First ensure its is in known state (seems odd but if you don't do this then the init fails)
        HAL_ADC_DeInit( adch );
        // setup init data
        adch->Init.Resolution            = ADC_RESOLUTION_12B;
        adch->Init.DataAlign             = ADC_DATAALIGN_RIGHT;
        adch->Init.ContinuousConvMode    = DISABLE;
        adch->Init.DiscontinuousConvMode = DISABLE;
        adch->Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
        adch->Init.ExternalTrigConv      = ADC_SOFTWARE_START;        
        adch->Init.DMAContinuousRequests = DISABLE;
        adch->Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
        adch->Init.NbrOfConversion       = 1;
        adch->Init.LowPowerAutoWait      = DISABLE;
        adch->Init.LowPowerAutoPowerOff  = DISABLE;
        int rc = HAL_ADC_Init( adch );
        if (rc!=HAL_OK) {
            return false;   // thats a fail
        }
        
        // Enable ADC1
        __HAL_ADC_ENABLE( adch );

        _adc1.active = true;
    }
    return true;
}

bool hal_bsp_adc_define(int pin, int chan) {
    ADC_HandleTypeDef* adch = &_adc1.adcHandle;
    ADC_ChannelConfTypeDef adcConf = { 0 };
    adcConf.Channel = chan;
    adcConf.Rank = ADC_REGULAR_RANK_1;
    adcConf.SamplingTime = ADC_SAMPLETIME_192CYCLES;

    int rc = HAL_ADC_ConfigChannel( adch, &adcConf );

    // If pin is 'real', define it as analog in 
    if (pin>=0 && pin <128) {
        /* 
        GPIO_InitTypeDef gpio_td = {
            .Pin = pin,
            .Mode = GPIO_MODE_ANALOG,
            .Pull = GPIO_NOPULL,
            .Alternate = pin
        };
        hal_gpio_init_stm(gpio_td.Pin, &gpio_td);
        */
        // easiest way is to deconfig it as GPIO as does the same without using stm specific hal fn
        hal_gpio_deinit(pin);
    }
    return (rc==HAL_OK);
}



int hal_bsp_adc_read(int channel) {
    ADC_HandleTypeDef* adch = &_adc1.adcHandle;
    ADC_ChannelConfTypeDef adcConf = { 0 };
    int adcData = 0;

    adcConf.Channel = channel;
    adcConf.Rank = ADC_REGULAR_RANK_1;
    adcConf.SamplingTime = ADC_SAMPLETIME_192CYCLES;
    HAL_ADC_ConfigChannel( adch, &adcConf );
    // Start ADC Software Conversion
    HAL_ADC_Start(adch);

    HAL_ADC_PollForConversion(adch, HAL_MAX_DELAY);

    adcData = HAL_ADC_GetValue(adch);
    
    return (uint16_t)adcData;
}

void hal_bsp_adc_release(int pin, int chan) {
    // noop? or define as ANALOG for lowest power?
    if (pin>=0 && pin <128) {
        // but this is already its state... 
        hal_gpio_deinit(pin);
    }

}
void hal_bsp_adc_deinit() {
    if (_adc1.active) {
        ADC_HandleTypeDef* adch = &_adc1.adcHandle;
        // Stop ADC
        __HAL_ADC_DISABLE( adch);
        // Stop its clock
        __HAL_RCC_ADC1_CLK_DISABLE( );

#if MYNEWT_VAL(STM32_CLOCK_HSE)
        // Disable HSI - no, required for system clock
        __HAL_RCC_HSI_DISABLE( );
#endif

        HAL_ADC_DeInit( adch );
        _adc1.active = false;
    }
}

#else   /* !ADC */

bool hal_bsp_adc_init() {
    return false;       // no ADC here
}

bool hal_bsp_adc_define(int pin, int chan) {
    return false;
}

int hal_bsp_adc_read(int channel) {
    return 0;
}

void hal_bsp_adc_release(int pin, int chan) {
}
void hal_bsp_adc_deinit() {
}
#endif  /* ADC */


void hal_bsp_uart_init(void)
{
#if MYNEWT_VAL(UART_0)
    __HAL_RCC_USART1_RELEASE_RESET( );
    __HAL_RCC_USART1_RELEASE_RESET( );
    __HAL_RCC_USART1_CLK_ENABLE( );

    hal_gpio_deinit(os_bsp_uart0_cfg.suc_pin_tx);
    hal_gpio_init_af(os_bsp_uart0_cfg.suc_pin_tx, os_bsp_uart0_cfg.suc_pin_af, 0, 0);
    
    hal_gpio_deinit(os_bsp_uart0_cfg.suc_pin_rx);
    hal_gpio_init_af(os_bsp_uart0_cfg.suc_pin_rx, os_bsp_uart0_cfg.suc_pin_af, 0, 0);
#endif
}


void hal_bsp_uart_deinit(void)
{
#if MYNEWT_VAL(UART_0)
    // Uart0 is UART1 in STM32 doc hence names of HAL defns
    GPIO_InitTypeDef highz_cfg = {
        .Mode = GPIO_MODE_ANALOG,
        .Pull = GPIO_NOPULL
    };

    __HAL_RCC_USART1_FORCE_RESET( );
    __HAL_RCC_USART1_RELEASE_RESET( );
    __HAL_RCC_USART1_CLK_DISABLE( );

    hal_gpio_deinit(os_bsp_uart0_cfg.suc_pin_tx);
    highz_cfg.Pin = os_bsp_uart0_cfg.suc_pin_tx;
    highz_cfg.Alternate = os_bsp_uart0_cfg.suc_pin_tx;
    hal_gpio_init_stm(highz_cfg.Pin, &highz_cfg);
    
    hal_gpio_deinit(os_bsp_uart0_cfg.suc_pin_rx);
    highz_cfg.Pin = os_bsp_uart0_cfg.suc_pin_rx;
    highz_cfg.Alternate = os_bsp_uart0_cfg.suc_pin_rx;
    hal_gpio_init_stm(highz_cfg.Pin, &highz_cfg);
#endif //MYNEWT_VAL(UART_0)
}

/** enter a MCU stop mode, with all periphs off or lowest possible power, and never return */
void hal_bsp_halt() {
      
    //tell lowpowermgr to deinit stuff
    hal_bsp_power_handler_sleep_enter(HAL_BSP_POWER_DEEP_SLEEP);

    // ask MCU HAL to stop it
    hal_mcu_halt();
}

#if MYNEWT_VAL(BSP_POWER_SETUP)

static LP_HOOK_t _hook_get_mode_cb=NULL;
static LP_HOOK_t _hook_exit_cb=NULL;
static LP_HOOK_t _hook_enter_cb=NULL;

/* hook idle enter/exit phases. 
 * Note the get_mode call is made with irq disabled in OS critical section - so don't hang about
 * enter/exit are called outside of the critical region
 * */
void hal_bsp_power_hooks(LP_HOOK_t getMode, LP_HOOK_t enter, LP_HOOK_t exit)
{
    // Should only have 1 hook of sleeping in the code, so assert if called twice
    assert(_hook_enter_cb==NULL);
    _hook_get_mode_cb = getMode;
    _hook_enter_cb = enter;
    _hook_exit_cb = exit;
}

/* the 2 following functions are called from hal_os_tick.c iff BSP_POWER_SETUP is set */
/* get the required power sleep mode that the application wants to be in */
int hal_bsp_power_handler_get_mode(os_time_t ticks)
{
    // ask to BSP for the appropriate power mode
    return (_hook_get_mode_cb!=NULL)?(*_hook_get_mode_cb)():HAL_BSP_POWER_WFI;
}

/* enter sleep - called before entering critical region */
void hal_bsp_power_handler_sleep_enter(int nextMode)
{
    if (_hook_enter_cb!=NULL) {
        (*_hook_enter_cb)();
    }

    /* Now BSP deinit                      */
    /* shared bus interfaces               */
    switch(nextMode) {
        
        case HAL_BSP_POWER_OFF:
        case HAL_BSP_POWER_DEEP_SLEEP:
        case HAL_BSP_POWER_SLEEP:
          
            /* I2S */
//            bsp_deinit_i2s();

            /* I2C */
            hal_bsp_deinit_i2c();

            /* SPI */
            hal_bsp_deinit_spi();

            /*UART */
            hal_bsp_uart_deinit();

            break;
        case HAL_BSP_POWER_WFI: 

        case HAL_BSP_POWER_ON:
        default:             
           break;
    }

}

/* exit sleep - called after exiting critical region */
void hal_bsp_power_handler_sleep_exit(int lastMode)
{
    /* and tell hook */
    if (_hook_exit_cb!=NULL) {
        (*_hook_exit_cb)();
    }

    /* Now BSP must reinit                 */
    /* shared bus interfaces               */
    switch(lastMode) {
        
        case HAL_BSP_POWER_OFF:
        case HAL_BSP_POWER_DEEP_SLEEP:
        case HAL_BSP_POWER_SLEEP:
     
            /* I2S */
//            bsp_init_i2s();

            /* I2C */         
            hal_bsp_init_i2c();

            /* SPI */
            hal_bsp_init_spi();

            /*UART */
            hal_bsp_uart_init();

            break;
        case HAL_BSP_POWER_WFI: 

        case HAL_BSP_POWER_ON:
        default:             
           break;
    }
}
#else 
void hal_bsp_power_hooks(LP_HOOK_t getMode, LP_HOOK_t enter, LP_HOOK_t exit) {
    // noop
    (void)getMode;
    (void)enter;
    (void)exit;
}
int hal_bsp_power_handler_get_mode(os_time_t ticks) {
    return HAL_BSP_POWER_WFI;
}
void hal_bsp_power_handler_sleep_enter(int nextMode){

}
void hal_bsp_power_handler_sleep_exit(int lastMode){
    
}

#endif
