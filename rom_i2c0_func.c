/*
 * rom_i2c0_func.c
 *
 *  Created on: 2015. 3. 8.
 *      Author: root
 */
#include "rom_i2c0_func.h"
void MasterErrCheck(uint8_t dev_addr, uint8_t reg_addr) {
    uint32_t err_num = ROM_I2CMasterErr(I2C0_BASE);
    switch (err_num) {
    case 0:
        return;
    case 16:
#ifdef UART
        UARTprintf("\nDevice = %x, Register = %x\nI2C_Arbitration Lost\n", dev_addr, reg_addr);
#endif
        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, RED_LED);
        exit(-1);
    case 4:
#ifdef UART
        UARTprintf("\nDevice = %x, Register = %x\nI2C_ADDRESS_ACK_ERROR\n", dev_addr, reg_addr);
#endif
        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, RED_LED);
        exit(-1);
    case 8:
#ifdef UART
        UARTprintf("\nDevice = %x, Register = %x\nI2C_DATACK_ERROR\n", dev_addr, reg_addr);
#endif
        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, RED_LED);
        exit(-1);
    case 12:
#ifdef UART
        UARTprintf("\nDevice = %x, Register = %x\nI2C_DATACK_ERROR AND I2C_ADDRESS_ACK_ERROR\n", dev_addr, reg_addr);
#endif
        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, RED_LED);
        exit(-1);
    default:
#ifdef UART
        UARTprintf("\nDevice = %x, Register = %x\nERROR_OCCUR\n", dev_addr, reg_addr);
#endif
        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, RED_LED);
        exit(-1);
    }
}
uint32_t I2C8bitRead(uint8_t dev_addr, uint8_t reg_addr) {
    ROM_I2CMasterSlaveAddrSet(I2C0_BASE, dev_addr, false);
    ROM_I2CMasterDataPut(I2C0_BASE, reg_addr);
    ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (ROM_I2CMasterBusy(I2C0_BASE));
    MasterErrCheck(dev_addr, reg_addr);
    ROM_I2CMasterSlaveAddrSet(I2C0_BASE, dev_addr, true);
    ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while (ROM_I2CMasterBusy(I2C0_BASE));
    MasterErrCheck(dev_addr, reg_addr);
    return ROM_I2CMasterDataGet(I2C0_BASE);
}
void I2CDirectSend(uint8_t dev_addr, uint8_t reg_addr) {
    ROM_I2CMasterSlaveAddrSet(I2C0_BASE, dev_addr, false);
    ROM_I2CMasterDataPut(I2C0_BASE, reg_addr);
    ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (ROM_I2CMasterBusy(I2C0_BASE));
    MasterErrCheck(dev_addr, 0xFF);
}
void I2C8bitWrite(uint8_t dev_addr, uint8_t reg_addr, uint8_t set_value) {
    ROM_I2CMasterSlaveAddrSet(I2C0_BASE, dev_addr, false);
    ROM_I2CMasterDataPut(I2C0_BASE, reg_addr);
    ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while (ROM_I2CMasterBusy(I2C0_BASE));
    MasterErrCheck(dev_addr, reg_addr);
    ROM_I2CMasterDataPut(I2C0_BASE, set_value);
    ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (ROM_I2CMasterBusy(I2C0_BASE));
    MasterErrCheck(dev_addr, reg_addr);
    }
uint32_t I2C24bitRead(uint8_t dev_addr, uint8_t reg_addr) {
    uint32_t data = 0;
    ROM_I2CMasterSlaveAddrSet(I2C0_BASE, dev_addr, false);
    ROM_I2CMasterDataPut(I2C0_BASE, reg_addr);
    ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (ROM_I2CMasterBusy(I2C0_BASE));
    MasterErrCheck(dev_addr, reg_addr);
    ROM_I2CMasterSlaveAddrSet(I2C0_BASE, dev_addr, true);
    ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while (ROM_I2CMasterBusy(I2C0_BASE));
    MasterErrCheck(dev_addr, reg_addr);
    ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while (ROM_I2CMasterBusy(I2C0_BASE));
    MasterErrCheck(dev_addr, reg_addr);
    data = ROM_I2CMasterDataGet(I2C0_BASE);
    ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while (ROM_I2CMasterBusy(I2C0_BASE));
    MasterErrCheck(dev_addr, reg_addr);
    data = data << 8 | ROM_I2CMasterDataGet(I2C0_BASE);
    ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while (ROM_I2CMasterBusy(I2C0_BASE));
    MasterErrCheck(dev_addr, reg_addr);
    return data << 8 | ROM_I2CMasterDataGet(I2C0_BASE);
}
uint32_t I2CDirect24bitRead(uint8_t dev_addr) {
    uint32_t data = 0;
    ROM_I2CMasterSlaveAddrSet(I2C0_BASE, dev_addr, true);
    ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while (ROM_I2CMasterBusy(I2C0_BASE));
    MasterErrCheck(dev_addr, 0xFF);
    ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while (ROM_I2CMasterBusy(I2C0_BASE));
    MasterErrCheck(dev_addr, 0xFF);
    data = ROM_I2CMasterDataGet(I2C0_BASE);
    ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while (ROM_I2CMasterBusy(I2C0_BASE));
    MasterErrCheck(dev_addr, 0xFF);
    data = data << 8 | ROM_I2CMasterDataGet(I2C0_BASE);
    ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while (ROM_I2CMasterBusy(I2C0_BASE));
    MasterErrCheck(dev_addr, 0xFF);
    return data << 8 | ROM_I2CMasterDataGet(I2C0_BASE);
}
uint16_t ReadPROM(uint8_t dev_addr, uint8_t subSlave_addr) {
    uint16_t PROM_Data = 0;
    I2CDirectSend(dev_addr, subSlave_addr);//address Set
    ROM_I2CMasterSlaveAddrSet(I2C0_BASE, dev_addr, true);//PROM read
    ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while (ROM_I2CMasterBusy(I2C0_BASE));
    MasterErrCheck(dev_addr, 0xFF);
    ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while (ROM_I2CMasterBusy(I2C0_BASE));
    MasterErrCheck(dev_addr, 0xFF);
    PROM_Data = ROM_I2CMasterDataGet(I2C0_BASE);
    ROM_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while (ROM_I2CMasterBusy(I2C0_BASE));
    MasterErrCheck(dev_addr, 0xFF);
    return PROM_Data << 8 | ROM_I2CMasterDataGet(I2C0_BASE);
}
