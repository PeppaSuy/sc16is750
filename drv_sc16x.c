/***************************************************
 * Module name: drv_sc16x.c
 *
 * Copyright 2015 - 2018, Mobike Company as an unpublished work.
 *
 * All Rights Reserved.
 *
 * The information contained herein is confidential
 * property of Mobike Company. The user, copying, transfer or
 * disclosure of such information is prohibited except
 * by express written agreement with Mobike Company.
 *
 * Module Description:
 * external flash sc16x16BSIG used SPI operation
 ***************************************************/
#include "drv_sc16x.h"
#include "drv_config.h"
#include "lib_i2c.h"
#include "lib_comfunc.h"
#include "stdio.h"
#include "RingBuffer.h"
#include "string.h"
#include "frm_debug.h"
#include "adpOS.h"
#include "nrf_drv_spi.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "drv_debug.h"
#include "nrfx_twim.h"

#define SC16X_USE_TWIM

#define SC16X_USE_I2C_MODE
#define SC16X_USE_FLOW_CONTROL

#define OUTPUT 1
#define INPUT  2

/* < SPI instance index. */
#define SC16X_SPI_INSTANCE  0 
static const nrfx_spim_t sc16x_spi = NRFX_SPIM_INSTANCE(SC16X_SPI_INSTANCE);  /**< SPI instance. */


#define SC16X_TWI_INSTANCE  1
static const nrfx_twim_t sc16x_twi = NRFX_TWIM_INSTANCE(SC16X_TWI_INSTANCE);


i2cCtrlBlock_s g_i2cSc16x;

#define SC16X_I2C_ADDR             SC16IS750_ADDRESS_AA
#define SC16X_TWI_ADDR             (SC16X_I2C_ADDR>>1)

int     peek_buf;       
uint8_t peek_flag;

static volatile bool m_twi_xfer_done;

static void sc16x_delay_ms (unsigned short ms)
{
    unsigned long i , j ;
    for(i=0;i< ms ;i++)
        for(j=0xffff;j>0;j--);
}

unsigned char sc16x_spi_send_char(uint8_t data)
{
	unsigned char rev_data = 0;
	unsigned long trans_end = 0 ;
    int ret = 0;
	nrfx_spim_xfer_desc_t s_data ;
	s_data.p_tx_buffer = &data ;
	s_data.tx_length = 1;
	s_data.p_rx_buffer = &rev_data;
	s_data.rx_length = 1;
    /*Enter critical for system may enter sleep mode while spi is waiting INT*/
    //taskENTER_CRITICAL();
	*(unsigned long *)nrfx_spim_end_event_get(&sc16x_spi) = 0;
    ret = nrfx_spim_xfer(&sc16x_spi, &s_data, NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER);
	trans_end = *(unsigned long *)nrfx_spim_end_event_get(&sc16x_spi);
	while(!trans_end )
	{
		trans_end = *(unsigned long *)nrfx_spim_end_event_get(&sc16x_spi);
	}
    //taskEXIT_CRITICAL();
    return rev_data;
}

int sc16x_spi_init(void)
{
    int ret = 0;
    nrfx_spim_config_t spi_config = NRFX_SPIM_DEFAULT_CONFIG;
    spi_config.ss_pin   = NRFX_SPIM_PIN_NOT_USED;
    spi_config.miso_pin = SC16X_SPI_MISO_PIN;
    spi_config.mosi_pin = SC16X_SPI_MOSI_PIN;
    spi_config.sck_pin  = SC16X_SPI_SCK_PIN;
    spi_config.frequency = NRF_SPIM_FREQ_1M;
    spi_config.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;
    spi_config.mode = NRF_SPIM_MODE_0;

    nrf_gpio_cfg_output(SC16X_SPI_NSS_PIN);
    
    nrfx_spim_uninit(&sc16x_spi);
    ret = nrfx_spim_init(&sc16x_spi, &spi_config, NULL, NULL);

    ret = ret == 0 ? 0 : -1;

    return ret;
}

void sc16x_twi_handler(nrfx_twim_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRFX_TWIM_EVT_DONE:
            if (p_event->xfer_desc.type == NRFX_TWIM_XFER_RX)
            {
               //NRF_LOG_INFO("twi rx done");// data_handler(m_sample);
            }
            else if(p_event->xfer_desc.type == NRFX_TWIM_XFER_TX)
            {
               // NRF_LOG_INFO("twi tx done");
            }
            m_twi_xfer_done = true;
            break;

        case NRFX_TWIM_EVT_ADDRESS_NACK:
             //NRF_LOG_INFO("Error event: NACK received after sending the address.");
            break;

        case NRFX_TWIM_EVT_DATA_NACK:
             //NRF_LOG_INFO("Error event: NACK received after sending a data byte.");
             break;

        default:
            break;
    }
}

int sc16x_twi_init(void)
{
    nrfx_err_t err_code = 0;
    nrfx_twim_config_t twi_config = NRFX_TWIM_DEFAULT_CONFIG;
    twi_config.scl = SC16X_I2C_SCL_PIN;
    twi_config.sda = SC16X_I2C_SDA_PIN;
    twi_config.frequency = NRF_TWIM_FREQ_400K;
    twi_config.interrupt_priority = APP_IRQ_PRIORITY_HIGH;
    twi_config.hold_bus_uninit = false;
    
    nrfx_twim_uninit(&sc16x_twi);
    err_code = nrfx_twim_init(&sc16x_twi, &twi_config, (nrfx_twim_evt_handler_t)sc16x_twi_handler, NULL);
    APP_ERROR_CHECK(err_code);
    nrfx_twim_enable(&sc16x_twi);

    return 0;
}

void sc16x_wait_done_evt(void)
{
    int index = 0;
    for(index = 0; index < 2048;index ++)
    {
        if(m_twi_xfer_done)
            break;
    }
}

void sc16x_reset_done_evt(void)
{
    m_twi_xfer_done = false;
}

void sc16x_check_twi_busy(void)
{
    int index = 0;
    for(index = 0; index < 1024;index ++)
    {
        if(!nrfx_twim_is_busy(&sc16x_twi))
            break;
    }
}


int sc16x_twiWrite(uint8_t wrAddr, uint8_t wrData)
{
    nrfx_err_t err_code;
	unsigned long trans_end = 0 ;
    uint8_t p_buf[3] = {0};
    p_buf[0] = wrAddr;
    p_buf[1] = wrData;
    sc16x_reset_done_evt();
    //while (nrfx_twim_is_busy(&sc16x_twi));
    sc16x_check_twi_busy();
	//*(unsigned long *)nrfx_twim_stopped_event_get(&sc16x_twi) = 0;
    err_code = nrfx_twim_tx(&sc16x_twi, SC16X_TWI_ADDR, p_buf, 2, false);
    //APP_ERROR_CHECK(err_code);
	//trans_end = *(unsigned long *)nrfx_twim_stopped_event_get(&sc16x_twi);
	//while(!trans_end )
	{
	//	trans_end = *(unsigned long *)nrfx_twim_stopped_event_get(&sc16x_twi);
	}
    sc16x_wait_done_evt(); 
    return 0;
}

int sc16x_twiRead(uint8_t rdAddr, uint8_t * rdData)
{
    nrfx_err_t err_code;
	unsigned long trans_end = 0 ;
    sc16x_reset_done_evt();
	//*(unsigned long *)nrfx_twim_stopped_event_get(&sc16x_twi) = 0;
    //while (nrfx_twim_is_busy(&sc16x_twi));
    sc16x_check_twi_busy();
    err_code = nrfx_twim_tx(&sc16x_twi, SC16X_TWI_ADDR, &rdAddr, 1, true);
    //APP_ERROR_CHECK(err_code);
    sc16x_wait_done_evt(); 
    sc16x_reset_done_evt();
    err_code = nrfx_twim_rx(&sc16x_twi, SC16X_TWI_ADDR, rdData, 1);
    //APP_ERROR_CHECK(err_code);
	//trans_end = *(unsigned long *)nrfx_twim_stopped_event_get(&sc16x_twi);
	//while(!trans_end )
	{
	//	trans_end = *(unsigned long *)nrfx_twim_stopped_event_get(&sc16x_twi);
	}
    sc16x_wait_done_evt(); 
    return 0;
}

int sc16x_i2cWrite(uint8_t wrAddr, uint8_t wrData)
{
    int8_t ret = -1;
    i2cCtrlBlock_s * cb = &g_i2cSc16x;

    /* start */
    ret = libI2C_start(cb);
    if (0 != ret)
    {
        return -1;
    }

    /* device addr + wr */
    libI2C_sendByte(cb, SC16X_I2C_ADDR | I2C_DIR_WR);
    ret = libI2C_waitACK(cb);
    if (0 != ret)
    {
        return -1;
    }

    libI2C_sendByte(cb, wrAddr);
    ret = libI2C_waitACK(cb);
    if (0 != ret)
    {
        return -2;
    }

    /* data */
    libI2C_sendByte(cb, wrData);
    ret = libI2C_waitACK(cb);
    if (0 != ret)
    {
        return -2;
    }

    /* stop */
    libI2C_stop(cb);

    return 0;
}

int sc16x_i2cRead(uint8_t rdAddr, uint8_t * rdData)
{
    int8_t ret = -1;
    i2cCtrlBlock_s * cb = &g_i2cSc16x;

    /* start */
    ret = libI2C_start(cb);
    if (0 != ret)
    {
        return -1;
    }

    /* device addr + wr */
    libI2C_sendByte(cb, SC16X_I2C_ADDR | I2C_DIR_WR);
    ret = libI2C_waitACK(cb);
    if (0 != ret)
    {
        return -1;
    }

    libI2C_sendByte(cb, rdAddr);
    ret = libI2C_waitACK(cb);
    if (0 != ret)
    {
        return -2;
    }

    /* start */
    ret = libI2C_start(cb);
    if (0 != ret)
    {
        return -2;
    }

    /* device addr + rd */
    libI2C_sendByte(cb, SC16X_I2C_ADDR | I2C_DIR_RD);
    ret = libI2C_waitACK(cb);
    if (0 != ret)
    {
        return -2;
    }

    /* read data */
    libI2C_recvByte(cb, rdData);
    libI2C_sendNACK(cb);

    /* STOP */
    libI2C_stop(cb);

    return 0;
}

void sc16x_i2c_init(void)
{
	g_i2cSc16x.SCL_pin = SC16X_I2C_SCL_PIN;
	g_i2cSc16x.SDA_pin = SC16X_I2C_SDA_PIN;
    libI2C_init(&g_i2cSc16x);
	
    /* reset i2c */
    libI2C_stop(&g_i2cSc16x);
    
    return;
}


void sc16x_int_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    sc16x_isrHandler();
}
 
int sc16x_Int_Init(void)
{
    uint32_t err_code = 0 ;
    if ( nrfx_gpiote_is_init () == 0) 
    {
        err_code = nrfx_gpiote_init();
    }
    nrf_drv_gpiote_in_config_t  config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);//     false true
    config.pull = NRF_GPIO_PIN_NOPULL;
    nrf_drv_gpiote_in_uninit(SC16X_INT_PIN);
    err_code = nrf_drv_gpiote_in_init(SC16X_INT_PIN, &config, sc16x_int_handler);
    if(err_code != NRFX_SUCCESS)
    {
        APP_ERROR_HANDLER(err_code);
    }
    nrf_drv_gpiote_in_event_enable(SC16X_INT_PIN, true); 
}

static void sc16x_activate(void)
{
    nrf_gpio_pin_clear(SC16X_SPI_NSS_PIN);
}

static void sc16x_deactivate(void)
{
    nrf_gpio_pin_set(SC16X_SPI_NSS_PIN);
}
 
int sc16x_available(void)
{
    return sc16x_fifoAvailableData();
}

int sc16x_read(void)
{
    int result = -1;
    uint8_t data;
    
	if ( peek_flag == 0) {
        result = sc16x_readByte(&data);
        if(result != -1)
            return data;
        else
            return result;
	} else {
		peek_flag = 0;
		return peek_buf;
	}
}

void sc16x_writeBytes(uint8_t *buf, uint8_t len)
{
    int index = 0;
    
    if(!sc16x_checkConfigValid())
    {
        sc16x_device_init();
        return;
    }
    
    for(index = 0; index < len; index ++)
    {
        sc16x_writeByte(buf[index]);
    }
}

void sc16x_pinMode(uint8_t pin, uint8_t i_o)
{
    sc16x_gpioSetPinMode(pin, i_o);
}

void sc16x_digitalWrite(uint8_t pin, uint8_t value)
{
    sc16x_gpioSetPinState(pin, value);
}

uint8_t sc16x_digitalRead(uint8_t pin)
{
   return sc16x_gpioGetPinState(pin);
}
 
uint8_t sc16x_readRegister(uint8_t reg)
{
	uint8_t command,ret;

	reg &= 0x0F;

#ifdef SC16X_USE_I2C_MODE
	command = reg << 3;
#ifdef SC16X_USE_TWIM
    sc16x_twiRead(command, &ret);
#else
    sc16x_i2cRead(command, &ret);
#endif
#else
	command = 0x80 | (reg << 3);
    sc16x_activate();
    //sc16x_delay_ms(10);
    sc16x_spi_send_char(command);
    //sc16x_delay_ms(10);
    ret = sc16x_spi_send_char(0xff);
    //sc16x_delay_ms(10);
    sc16x_deactivate();
    //sc16x_delay_ms(10);
#endif
	return ret;
}
 
void sc16x_writeRegister(uint8_t reg, uint8_t value)
{
	uint8_t command;

#ifdef SC16X_USE_I2C_MODE
    command = reg << 3;
#ifdef SC16X_USE_TWIM
    sc16x_twiWrite(command, value);
#else
    sc16x_i2cWrite(command, value);
#endif
#else
	reg &= 0x0F;
	command = 0x00 | (reg << 3);

	sc16x_activate();
    //sc16x_delay_ms(10);
	sc16x_spi_send_char(command);
    //sc16x_delay_ms(10);
	sc16x_spi_send_char(value);
    //sc16x_delay_ms(10);
	sc16x_deactivate();
    //sc16x_delay_ms(10);
#endif
}

int16_t  sc16x_setBaudrate(uint32_t baudrate)
{
        uint16_t divisor;
        uint8_t prescaler;
        uint32_t actual_baudrate;
        int16_t error;
        uint8_t temp_lcr;
        if ( (sc16x_readRegister(SC16IS750_REG_MCR)&0x80) == 0) { //if prescaler==1
            prescaler = 1;
        } else {
            prescaler = 4;
        }
    
        divisor = (SC16IS750_CRYSTCAL_FREQ/prescaler)/(baudrate*16);
    
        temp_lcr = sc16x_readRegister(SC16IS750_REG_LCR);
        temp_lcr |= 0x80;
        sc16x_writeRegister(SC16IS750_REG_LCR,temp_lcr);
        //write to DLL
        sc16x_writeRegister(SC16IS750_REG_DLL,(uint8_t)divisor);
        //write to DLH
        sc16x_writeRegister(SC16IS750_REG_DLH,(uint8_t)(divisor>>8));
        temp_lcr &= 0x7F;
        sc16x_writeRegister(SC16IS750_REG_LCR,temp_lcr);
    
    
        actual_baudrate = (SC16IS750_CRYSTCAL_FREQ/prescaler)/(16*divisor);
        error = ((float)actual_baudrate-baudrate)*1000/baudrate;
        
        return error;
}

bool sc16x_checkConfigValid(void)
{
    uint8_t temp_lcr;
    temp_lcr = sc16x_readRegister(SC16IS750_REG_LCR);
    if((temp_lcr & 0x03) == 3)
    {
        return true;
    }
    else
    {
        return false;
    }
}


void sc16x_setLine(uint8_t data_length, uint8_t parity_select, uint8_t stop_length )
{
    uint8_t temp_lcr;
    temp_lcr = sc16x_readRegister(SC16IS750_REG_LCR);
    temp_lcr &= 0xC0; //Clear the lower six bit of LCR (LCR[0] to LCR[5]

    switch (data_length) {
        case 5:
            break;
        case 6:
            temp_lcr |= 0x01;
            break;
        case 7:
            temp_lcr |= 0x02;
            break;
        case 8:
            temp_lcr |= 0x03;
            break;
        default:
            temp_lcr |= 0x03;
            break;
    }

    if ( stop_length == 2 ) {
        temp_lcr |= 0x04;
    }

    switch (parity_select) {            //parity selection length settings
        case 0:                         //no parity
             break;
        case 1:                         //odd parity
            temp_lcr |= 0x08;
            break;
        case 2:                         //even parity
            temp_lcr |= 0x18;
            break;
        case 3:                         //force '1' parity
            temp_lcr |= 0x03;
            break;
        case 4:                         //force '0' parity
            break;
        default:
            break;
    }

    sc16x_writeRegister(SC16IS750_REG_LCR,temp_lcr);
}

void sc16x_gpioSetPinMode(uint8_t pin_number, uint8_t i_o)
{
    uint8_t temp_iodir;

    temp_iodir = sc16x_readRegister(SC16IS750_REG_IODIR);
    if ( i_o == OUTPUT ) {
      temp_iodir |= (0x01 << pin_number);
    } else {
      temp_iodir &= (uint8_t)~(0x01 << pin_number);
    }

    sc16x_writeRegister(SC16IS750_REG_IODIR, temp_iodir);
    return;
}

void sc16x_gpioSetPinState(uint8_t pin_number, uint8_t pin_state)
{
    uint8_t temp_iostate;

    temp_iostate = sc16x_readRegister(SC16IS750_REG_IOSTATE);
    if ( pin_state == 1 ) {
      temp_iostate |= (0x01 << pin_number);
    } else {
      temp_iostate &= (uint8_t)~(0x01 << pin_number);
    }

    sc16x_writeRegister(SC16IS750_REG_IOSTATE, temp_iostate);
    return;
}

uint8_t sc16x_gpioGetPinState(uint8_t pin_number)
{
    uint8_t temp_iostate;

    temp_iostate = sc16x_readRegister(SC16IS750_REG_IOSTATE);
    if ( temp_iostate & (0x01 << pin_number)== 0 ) {
      return 0;
    }
    return 1;
}

uint8_t sc16x_gpioGetPortState(void)
{
    return sc16x_readRegister(SC16IS750_REG_IOSTATE);
}

void sc16x_gpioSetPortMode(uint8_t port_io)
{
    sc16x_writeRegister(SC16IS750_REG_IODIR, port_io);
    return;
}

void sc16x_gpioSetPortState(uint8_t port_state)
{
    sc16x_writeRegister(SC16IS750_REG_IOSTATE, port_state);
    return;
}

void sc16x_setPinInterrupt(uint8_t io_int_ena)
{
    sc16x_writeRegister(SC16IS750_REG_IOINTENA, io_int_ena);
    return;
}

void sc16x_resetDevice(void)
{
    uint8_t reg;

    reg = sc16x_readRegister(SC16IS750_REG_IOCONTROL);
    reg |= 0x08;
    sc16x_writeRegister(SC16IS750_REG_IOCONTROL, reg);

    return;
}

void sc16x_modemPin(uint8_t gpio) //gpio == 0, gpio[7:4] are modem pins, gpio == 1 gpio[7:4] are gpios
{
    uint8_t temp_iocontrol;

    temp_iocontrol = sc16x_readRegister(SC16IS750_REG_IOCONTROL);
    if ( gpio == 0 ) {
        temp_iocontrol |= 0x02;
    } else {
        temp_iocontrol &= 0xFD;
    }
    sc16x_writeRegister(SC16IS750_REG_IOCONTROL, temp_iocontrol);

    return;
}

void sc16x_gpioLatch(uint8_t latch)
{
    uint8_t temp_iocontrol;

    temp_iocontrol = sc16x_readRegister(SC16IS750_REG_IOCONTROL);
    if ( latch == 0 ) {
        temp_iocontrol &= 0xFE;
    } else {
        temp_iocontrol |= 0x01;
    }
    sc16x_writeRegister(SC16IS750_REG_IOCONTROL, temp_iocontrol);

    return;
}

void sc16x_interruptControl(uint8_t int_ena)
{
    sc16x_writeRegister(SC16IS750_REG_IER, int_ena);
}

uint8_t sc16x_InterruptPendingTest(void)
{
    return (sc16x_readRegister(SC16IS750_REG_IIR) & 0x01);
}

void sc16x_isrHandler(void)
{
    uint8_t irq_src;  
    uint8_t rdByte = -1;
    int result = -1;
    
    irq_src = sc16x_readRegister(SC16IS750_REG_IIR);
    irq_src = (irq_src >> 1);
    irq_src &= 0x3F;
    
    //printf("sc16x_isrHandler irq_src = 0x%x\r\n", irq_src);

    switch (irq_src) {
        case 0x06:                  //Receiver Line Status Error
            break;
        case 0x0c:               //Receiver time-out interrupt
            break;
        case 0x04:               //RHR interrupt
            result = sc16x_readByte(&rdByte);
            if(result != -1)
            {
                TransforByteToRingBuffer(rdByte);
            }
            break;
        case 0x02:               //THR interrupt
            break;
        case 0x00:                  //modem interrupt;
            break;
        case 0x30:                  //input pin change of state
            break;
        case 0x10:                  //XOFF
            break;
        case 0x20:                  //CTS,RTS
            break;
        default:
            break;
    }
    return;
}

void sc16x_fifo_control(uint8_t fifo_ctrl)
{
    sc16x_writeRegister(SC16IS750_REG_FCR,fifo_ctrl);
    return;
}

void sc16x_fifoSetTriggerLevel(uint8_t value)
{
    uint8_t temp_reg;

    temp_reg = sc16x_readRegister(SC16IS750_REG_MCR);
    temp_reg |= 0x04;
    sc16x_writeRegister(SC16IS750_REG_MCR,temp_reg); //SET MCR[2] to '1' to use TLR register or trigger level control in FCR register

    temp_reg = sc16x_readRegister(SC16IS750_REG_EFR);
    //sc16x_writeRegister(SC16IS750_REG_EFR, temp_reg|0x10); //set ERF[4] to '1' to use the  enhanced features
    sc16x_writeRegister(SC16IS750_REG_TLR, value); //Tx FIFO trigger level setting
    //sc16x_writeRegister(SC16IS750_REG_EFR, temp_reg); //restore EFR register

    return;
}

void sc16x_flowControl(uint8_t value)
{
    uint8_t temp_reg;

    sc16x_writeRegister(SC16IS750_REG_LCR,0xBF); //SET MCR[2] to '1' to use TLR register or trigger level control in FCR register

    temp_reg = sc16x_readRegister(SC16IS750_REG_EFR);
    sc16x_writeRegister(SC16IS750_REG_EFR, temp_reg | 0xC0); //set ERF[4] to '1' to use the  enhanced features

    return;
}


uint8_t sc16x_fifoAvailableData(void)
{
   return sc16x_readRegister(SC16IS750_REG_RXLVL);
//    return sc16x_readRegister(SC16IS750_REG_LSR) & 0x01;
}

uint8_t sc16x_fifoAvailableSpace(void)
{
   return sc16x_readRegister(SC16IS750_REG_TXLVL);

}

void sc16x_writeByte(uint8_t val)
{
	uint8_t tmp_lsr;
    #if 0
    while ( sc16x_fifoAvailableSpace() == 0 ){
	};
    sc16x_writeRegister(SC16IS750_REG_THR,val);
    #else
	sc16x_writeRegister(SC16IS750_REG_THR,val);

    /*
     * Fixed by Lin, Must comment the while(), because this function is not thread safe,
     * SC16IS750_REG_LSR may be change by other thread, and the code can't read correcet value,
     * so system may not break from the loop
     */
	/* do { */
		/* tmp_lsr = sc16x_readRegister(SC16IS750_REG_LSR); */
        /* if (++count > 500) */
            /* break; */
	/* } while ((tmp_lsr&0x20) == 0); */
    #endif
}

int sc16x_readByte(uint8_t *data)
{
	volatile uint8_t val;
	if (sc16x_fifoAvailableData() == 0) {
		return -1;

	} else {
	  *data = sc16x_readRegister(SC16IS750_REG_RHR);
      return 1;
	}
}

void sc16x_enableTransmit(uint8_t tx_enable)
{
    uint8_t temp_efcr;
    temp_efcr = sc16x_readRegister(SC16IS750_REG_EFCR);
    if ( tx_enable == 0) {
        temp_efcr |= 0x04;
    } else {
        temp_efcr &= 0xFB;
    }
    sc16x_writeRegister(SC16IS750_REG_EFCR,temp_efcr);

    return;
}

void sc16x_enableReceive(uint8_t rx_enable)
{
    uint8_t temp_efcr;
    temp_efcr = sc16x_readRegister(SC16IS750_REG_EFCR);
    if ( rx_enable == 0) {
        temp_efcr |= 0x02;
    } else {
        temp_efcr &= 0xFD;
    }
    sc16x_writeRegister(SC16IS750_REG_EFCR,temp_efcr);

    return;
}


uint8_t sc16x_ping(void)
{
	sc16x_writeRegister(SC16IS750_REG_SPR,0x55);
	if (sc16x_readRegister(SC16IS750_REG_SPR) !=0x55) {
        LOG_DEBUG_PRINT("SC16IS750_REG_SPR error,  0x55 != 0x%x\r\n", sc16x_readRegister(SC16IS750_REG_SPR));
		return 0;
	}

    #if 0
	sc16x_writeRegister(SC16IS750_REG_SPR,0xAA);
	if (sc16x_readRegister(SC16IS750_REG_SPR) !=0xAA) {
        LOG_DEBUG_PRINT("SC16IS750_REG_SPR error, 0xAA != 0x%x\r\n", sc16x_readRegister(SC16IS750_REG_SPR));
		return 0;
	}
    #endif

	return 1;
}
/*
void sc16x_setTimeout(uint32_t time_out)
{
	timeout = time_out;
}
size_t sc16x_readBytes(char *buffer, size_t length)
{
	size_t count=0;
	int16_t tmp;
	while (count < length) {
		tmp = readwithtimeout();
		if (tmp < 0) {
			break;
		}
		*buffer++ = (char)tmp;
		count++;
	}
	return count;
}
int16_t sc16x_readwithtimeout()
{
  int16_t tmp;
  uint32_t time_stamp;
  time_stamp = millis();
  do {
    tmp = read();
    if (tmp >= 0) return tmp;
  } while(millis() - time_stamp < timeout);
  return -1;     // -1 indicates timeout
}
*/

void sc16x_flush(void)
{
	uint8_t tmp_lsr;

	do {
		tmp_lsr = sc16x_readRegister(SC16IS750_REG_LSR);
	} while ((tmp_lsr&0x20) ==0);


}

int sc16x_peek(void)
{
    int result = -1;
    uint8_t data;
    
	if ( peek_flag == 0) {
        result = sc16x_readByte(&peek_buf);
        if(result != -1)
			peek_flag = 1;
       }

	return peek_buf;
}

int try_to_get_char(uint8_t *input)
{
    int result = -1;
    uint8_t reg_value = 0;
    uint8_t data;

    static int try_count = 0;

    if(!sc16x_checkConfigValid())
    {
        try_count ++;
        if(try_count > 100)
        {
            try_count = 0;
            sc16x_device_init();
        }
        return -1;
    }
    
    try_count = 0;
    
    //reg_value = sc16x_readRegister(SC16IS750_REG_LSR);
    //if(reg_value & 0x01)
    {
        result = sc16x_readByte(&data);
    }

    if(result != -1)
    {
        //printf("LSR:0x%x, data:0x%x\r\n", reg_value, data);
        *input = data;
    }
    
Exit0:
    return result;
}

int sc16x_device_init(void)
{
    peek_flag = 0;
#ifdef SC16X_USE_I2C_MODE
#ifdef SC16X_USE_TWIM
    sc16x_twi_init();
#else
    sc16x_i2c_init();
#endif
#else
    sc16x_spi_init();
#endif
    sc16x_resetDevice();
    sc16x_enableTransmit(1);
    sc16x_enableReceive(1);
#ifdef SC16X_USE_FLOW_CONTROL
    sc16x_flowControl(0xC0);
#endif
    sc16x_setBaudrate(115200);
    sc16x_setLine(8,0,1);
    sc16x_fifoSetTriggerLevel(0x00);
    sc16x_fifo_control(0x6);
    sc16x_fifo_control(0xf1);
    sc16x_interruptControl(0);
    //sc16x_Int_Init();
}

