//
// rom_i2c0_func.h
//
//  Created on: 2015. 3. 8.
//      Author: root
//

#ifndef ROM_I2C0_FUNC_H_
#define ROM_I2C0_FUNC_H_

#define UART

#define TARGET_IS_BLIZZARD_RA1
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "driverlib/i2c.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"

//////////// Define pin to LED color mapping ////////////
#define RED_LED   GPIO_PIN_1
#define BLUE_LED  GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3
#define I2C0_BASE               0x40020000  // I2C0

void MasterErrCheck(uint8_t dev_addr, uint8_t reg_addr);
uint32_t I2C8bitRead(uint8_t dev_addr, uint8_t reg_addr);
void I2C8bitWrite(uint8_t dev_addr, uint8_t reg_addr, uint8_t set_value);
void I2CDirectsend(uint8_t dev_addr, uint8_t reg_addr);
uint32_t I2C24bitRead(uint8_t dev_addr, uint8_t reg_addr);
uint32_t I2CDirect24bitRead(uint8_t dev_addr);
uint16_t ReadPROM(uint8_t dev_addr, uint8_t subSlave_addr);

#endif /* ROM_I2C0_FUNC_H_ */
