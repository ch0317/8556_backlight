#ifndef __BSP_I2C_EE_H
#define	__BSP_I2C_EE_H


#include "stm32f10x.h"


#define WRITE_DIR_8556 		0x58
#define READ_DIR_8556 		0x59

#define ADDR_8556					0x2c


#define BRIGHTNESS_CONTROL 0x00
#define DEVICE_CONTROL     0x01
#define STATUS             0x02
#define IDENTIFICATION     0x03
#define DIRECT_CONTROL     0x04

#define LED_ENABLE                 0x16

#define  CFG98  0x98 
#define  CFG9E  0x9E 
#define  CFG0   0xA0 
#define  CFG1   0xA1 
#define  CFG2   0xA2 
#define  CFG3   0xA3 
#define  CFG4   0xA4 
#define  CFG5   0xA5 
#define  CFG6   0xA6 
#define  CFG7   0xA7 
#define  CFG8   0xA8 
#define  CFG9   0xA9 
#define  CFGA   0xAA 
#define  CFGB   0xAB 
#define  CFGC   0xAC 
#define  CFGD   0xAD 
#define  CFGE   0xAE 
#define  CFGF   0xAF 

uint8_t ee_CHECK_DEVICE(uint8_t addr);
uint8_t ee_WRITE_BYTES(uint8_t w_addr,uint8_t *data,uint16_t size);
uint8_t ee_READ_BYTES(uint8_t r_addr,uint8_t *data,uint16_t size);

#endif /* __BSP_I2C_EE_H */
