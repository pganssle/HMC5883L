/** @file
Library for reading and writing to and from registers on I2C devices.

This code is released under a Creative Commons Attribution 4.0 International license 
([CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/)).

@author Paul J. Ganssle
@version 0.1
@date 2015-01-14
*/

#include <Wire.h>
#include <I2CDev.h>

I2CDev::I2CDev(uint8_t address) {
    /** I2C device class constructor

    */
    dev_addr = address;
}

void I2CDev::start() {
    /** Start I2C communication with the specified device. Alias for `Wire.begin()`. */

    Wire.begin();
}

uint8_t I2CDev::write_data(uint8_t register_addr, uint8_t data) {
    /** Writes data to the specified register address.

    This is a private function, called by specific-use functions such as `write_RDAC()` and 
    `write_EEMEM()` to write data (specified by `data`) into the register specified by 
    `register_addr` using the Wire.h library.

    @param register_addr The register address to query.
    @param data The data to write to the specified address.

    @return Returns 0 on no error, otherwise returns I2C errors:
            - \c `EC_NO_ERR`: No error.
            - \c `EC_DATA_LONG`: Data too long to fit in transmit buffer
            - \c `EC_NACK_ADDR`: Received NACK on transmit of address.
            - \c `EC_NACK_DATA`: Received NACK on transmit of data.
            - \c `EC_I2C_OTHER`: Other I2C error.
    */

    Wire.beginTransmission(dev_addr);
    Wire.write(register_addr);
    Wire.write(data);
    err_code = Wire.endTransmission();
    return err_code;
}

uint8_t *I2CDev::read_data(uint8_t register_addr, uint8_t length) {
    /** Reads data of length `length` from register  `register_addr`
    
    This is a private function, called by specific-use functions such as `read_RDAC()` and 
    `read_EEMEM()` to read a data array of length `length` (in bytes) from the register specified by
    `register_addr`.

    @param[in] register_addr The address of the register to read from.
    @param[in] length The length of the data stored in the register.

    @return On success, returns a pointer to an array of `uint8_t` unsigned bytes retrieved from the
            register. The returned array has length `length`, which in most cases will be 1. On 
            error, this function returns a `NULL` pointer and sets `err_code` (query 
            `get_err_code()` to get the value of this variable) to one of the I2C errors:
            - \c `EC_NO_ERR`: No error.
            - \c `EC_DATA_LONG`: Data too long to fit in transmit buffer
            - \c `EC_NACK_ADDR`: Received NACK on transmit of address.
            - \c `EC_NACK_DATA`: Received NACK on transmit of data.
            - \c `EC_I2C_OTHER`: Other I2C error.
    */

    uint8_t error = 0;

    Wire.beginTransmission(dev_addr);
    Wire.write(register_addr);
    err_code = Wire.endTransmission();
    if(err_code > 0) {
        return NULL;
    }

    Wire.beginTransmission(dev_addr);
    uint8_t n_bytes = Wire.requestFrom(dev_addr, length);

    if(n_bytes != length) {
        err_code = EC_BAD_READ_SIZE;
        return NULL;
    }

    uint8_t buff[length];
    if(Wire.available() == length) {
        for(int i = 0; i < length; i++) {
            buff[i] = Wire.read();
        }
    }

    err_code = Wire.endTransmission();
    if(err_code > 0) {
        return NULL;
    }

    return buff;
}

uint8_t I2CDev::read_data_byte(uint8_t register_addr) {
    /** Reads a single byte from the specified register. Convenience wrapper for `read_data()`.

    This reads a single byte from the register specified at `register_addr` via a call to
    `read_data()`. In most cases, only a single byte is stored at the specified register address.

    @param[in] register_addr The register address from which to read a single byte.
    
    @return Returns the requested value. If there is an error, returns 0 and `err_code` (retrieved
    via `get_err_code()`) is set to a non-zero value. This is a simple wrapper for `read_data()`, so
    it raises only the errors raised by that function.
    */

    uint8_t *rv = read_data(dev_addr, register_addr, 1);

    if(rv == NULL) {
        return 0;       // Err code set in read_data already.
    }

    return rv[0];
}

uint8_t I2CDev::get_err_code() {
    /** Retrieve the error code stored on the I2C device.
    
    Retrieves the error code stored in `err_code` variable. If non-zero, use `I2CGetErrorString()`
    from `I2CErrorStrings.h` to retrieve the human-readable error string, or see `I2CDev.h` for
    details.

    @return Returns the error code set in the current object. Non-zero value is an error.
    */
    return err_code;
}