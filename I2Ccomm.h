/** @file
Library for reading and writing to and from registers on I2C devices.

@author Paul J. Ganssle
@version 0.1
@date 2015-01-14
*/

#ifndef I2CCOMM_H
#define I2CCOMM_H

#define EC_NO_ERR 0
#define EC_DATA_LONG 1
#define EC_NACK_ADDR 2
#define EC_I2C_OTHER 3

uint8_t write_data(uint8_t dev_addr, uint8_t register_addr, uint8_t data);
uint8_t read_data(uint8_t dev_addr, uint8_t register_addr, uint8_t length);
uint8_t read_data_byte(uint8_t dev_addr, uint8_t register_addr, uint8_t data);

#endif