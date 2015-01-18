/** @file
Library for reading and writing to and from registers on I2C devices.

This code is released under a Creative Commons Attribution 4.0 International license 
([CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/)).

@author Paul J. Ganssle
@version 0.1
@date 2015-01-14
*/

#ifndef I2CDEV_H
#define I2CDEV_H

#define EC_NO_ERR 0
#define EC_DATA_LONG 1
#define EC_NACK_ADDR 2
#define EC_I2C_OTHER 3

class I2CDev {
public:
    I2CDev(uint8_t address);

    uint8_t start(void);

    uint8_t write_data(uint8_t register_addr, uint8_t data);
    uint8_t *read_data(uint8_t register_addr, uint8_t length);
    uint8_t read_data_byte(uint8_t register_addr, uint8_t data);

    uint8_t get_err_code(void);
private:
    I2CDev() : err_code(0) {}

    uint8_t err_code;
    uint8_t dev_addr;
};

#endif