/******************** (C) COPYRIGHT 2015 MoBike Technology *********************
* File Name          : Drivers/Include/drv_ble.h
* Author             : Mobike Embedded Software Team
* Date First Issued  : 2016/1/25 : Version 0.1
* Description        :
********************************************************************************
* History:
* 2017/1/25 : Version 0.1 Stuart
*
******************************************************************************
* @attention
*
* TODO:here need to add copyright & opensource GPL
*
*******************************************************************************/


#ifndef DRV_SPI2UART_H
#define DRV_SPI2UART_H

#include "drv_exti.h"
#include "stdint.h"
#include "adpHeader.h"

#define SC16X_WRITE_ENABLE_CMD          0x06
#define SC16X_WRITE_DISABLE_CMD         0x04
#define SC16X_4K_SECTOR_ERASE_CMD       0X20 
#define SC16X_32K_BLOCK_ERASE_CMD       0X52
#define SC16X_CHIP_ERASE_CMD            0xC7 
#define SC16X_PAGE_PROGRAM              0x02
#define SC16X_READ_ID                   0x9F
#define SC16X_READ_STATUS_REG           0x05
#define SC16X_READ_DATA                 0x03

//A:VDD
//B:GND
//C:SCL
//D:SDA
#define     SC16IS750_ADDRESS_AA     (0X90)
#define     SC16IS750_ADDRESS_AB     (0X92)
#define     SC16IS750_ADDRESS_AC     (0X94)
#define     SC16IS750_ADDRESS_AD     (0X96)
#define     SC16IS750_ADDRESS_BA     (0X98)
#define     SC16IS750_ADDRESS_BB     (0X9A)
#define     SC16IS750_ADDRESS_BC     (0X9C)
#define     SC16IS750_ADDRESS_BD     (0X9E)
#define     SC16IS750_ADDRESS_CA     (0XA0)
#define     SC16IS750_ADDRESS_CB     (0XA2)
#define     SC16IS750_ADDRESS_CC     (0XA4)
#define     SC16IS750_ADDRESS_CD     (0XA6)
#define     SC16IS750_ADDRESS_DA     (0XA8)
#define     SC16IS750_ADDRESS_DB     (0XAA)
#define     SC16IS750_ADDRESS_DC     (0XAC)
#define     SC16IS750_ADDRESS_DD     (0XAE)


//General Registers
#define     SC16IS750_REG_RHR        (0x00)
#define     SC16IS750_REG_THR        (0X00)
#define     SC16IS750_REG_IER        (0X01)
#define     SC16IS750_REG_FCR        (0X02)
#define     SC16IS750_REG_IIR        (0X02)
#define     SC16IS750_REG_LCR        (0X03)
#define     SC16IS750_REG_MCR        (0X04)
#define     SC16IS750_REG_LSR        (0X05)
#define     SC16IS750_REG_MSR        (0X06)
#define     SC16IS750_REG_SPR        (0X07)
#define     SC16IS750_REG_TCR        (0X06)
#define     SC16IS750_REG_TLR        (0X07)
#define     SC16IS750_REG_TXLVL      (0X08)
#define     SC16IS750_REG_RXLVL      (0X09)
#define     SC16IS750_REG_IODIR      (0X0A)
#define     SC16IS750_REG_IOSTATE    (0X0B)
#define     SC16IS750_REG_IOINTENA   (0X0C)
#define     SC16IS750_REG_IOCONTROL  (0X0E)
#define     SC16IS750_REG_EFCR       (0X0F)

//Special Registers
#define     SC16IS750_REG_DLL        (0x00)
#define     SC16IS750_REG_DLH        (0X01)

//Enhanced Registers
#define     SC16IS750_REG_EFR        (0X02)
#define     SC16IS750_REG_XON1       (0X04)
#define     SC16IS750_REG_XON2       (0X05)
#define     SC16IS750_REG_XOFF1      (0X06)
#define     SC16IS750_REG_XOFF2      (0X07)

//
#define     SC16IS750_INT_CTS        (0X80)
#define     SC16IS750_INT_RTS        (0X40)
#define     SC16IS750_INT_XOFF       (0X20)
#define     SC16IS750_INT_SLEEP      (0X10)
#define     SC16IS750_INT_MODEM      (0X08)
#define     SC16IS750_INT_LINE       (0X04)
#define     SC16IS750_INT_THR        (0X02)
#define     SC16IS750_INT_RHR        (0X01)

#define     SC16IS750_CRYSTCAL_FREQ (14745600UL) 

int sc16x_readByte(uint8_t *data);
uint8_t sc16x_gpioGetPinState(uint8_t pin_number);
uint8_t sc16x_fifoAvailableData(void);
uint8_t sc16x_ping(void);
void sc16x_writeByte(uint8_t val);
void sc16x_writeBytes(uint8_t *buf, uint8_t len);
int sc16x_device_init(void);
int try_to_get_char(uint8_t *input);
bool sc16x_checkConfigValid(void);

#endif
