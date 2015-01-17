/** @file
Class file for HMC5883L three-axis magnetometer primary interface to Arduino over I2C.

This code is released under a Creative Commons Attribution 4.0 International license 
([CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/)).

@author Paul J. Ganssle
@version 0.1
@date 2015-01-14
*/

#include <HMC5883L.h>
#include <I2CDev.h>
#include <Arduino.h>

HMC5883L::HMC5883L() {
    /**  Constructor for HMC5883L compass / magnetometer class. */
    I2CDevice = I2CDev(HMC5883L_ADDR);
}

uint8_t HMC5883L::initialize(bool noConfig=false) {
    /** Initialize the magnetometer communications.

    Starts I2C communication with the HMC5883L magnetometer. This creates the Wire interface
    with the device. If the `noConfig` parameter is set to `false` (default), the magnetometer
    is also explicitly initialized with the device default values for the various configuration
    parameters:
    
    | Parameter        | Default value                                            |
    | :--------------- | :------------------------------------------------------- |
    | Gain             | `[HMC_GAIN130]` 1090 LSb/Gauss (+/- 1.3 G sensor range)  |
    | Averaging rate   | `[HMC_AVG1]` 1 point per sample                          |
    | Data output rate | `[HMC_RATE1500]` 15 Hz                                   |
    | Measurement mode | `[HMC_MeasurementIdle]` Idle mode                        |
    | Bias mode        | `[HMC_BIAS_NONE]` No bias                                |

    If `noConfig` is set to `true`, this will request the values of some of the parameters
    already set, so as to ensure the accuracy of calls to functions such as `getDelay()`, which
    may use cached values for compass parameters.

    @param[in] noConfig Optional parameter. If specified `true`, explicitly initializes the
                        device using default parameters.

    @return Returns `0` on no error, or an error code. See `HMC5883L_Errors.h` for details.
    */

    // Start communication with the device.
    I2CDevice.start();

    uint8_t rv;
    if (!noConfig) {
        // Setup the configuration,.
        if (rv = setGain(HMC_GAIN130)) {
            err_code = rv;
            return rv;
        }

        if (rv = setAveragingRate(HMC_AVG1)) {
            err_code = rv;
            return rv;
        }

        if (rv = setOutputRate(HMC_RATE1500)) {
            err_code = rv;
            return rv;
        }

        if (rv = setMeasurementMode(HMC_MeasurementIdle)) {
            err_code = rv;
            return rv;
        }

        if (rv = setBiasMode(HMC_BIAS_NONE)) {
            err_code = rv;
            return rv;
        }
    } else {
        // Cache the values for the existing settings.
        if (!(rv = getGain(true))) {
            if (err_code) { return err_code; }
        }

        if (!(rv = getAveragingRate(true))) {
            if (err_code) { return err_code; }
        }

        if (!(rv = getOutputRate(true))) {
            if (err_code) { return err_code; }
        }

        if (!(rv = getMeasurementMode(true))) {
            if (err_code) { return err_code; }
        }

        if (!(rv = getBiasMode(true))) {
            if (err_code) { return err_code; }
        }
    }

    return rv;
}

uint8_t HMC5883L::setGain(uint8_t gain_level) {
    /** Set the magnetometer gain value.

    Sets the magnetometer gain value, which determines the sensor range  and the digital
    resolution. The gain level is bits 5-7 of Configuration Register B. The rest of configuration
    register B should be 0.

    @param[in] gain_level The gain setting, the values for which can be found in the following
                          table:

    | `gain_level`  | Value  | Gain (LSB/G) | Range (G) | Resolution (mG / LSB) |
    | :------------ | :----: | :----------: | :-------: | :-------------------: |
    | `HMC_GAIN088` |    0   |    1370      |   ±0.88   |        0.73           |
    | `HMC_GAIN130` |    1   |    1090      |   ±1.30   |        0.92           |
    | `HMC_GAIN190` |    2   |     820      |   ±1.90   |        1.22           |
    | `HMC_GAIN250` |    3   |     660      |   ±2.50   |        1.52           |
    | `HMC_GAIN400` |    4   |     440      |   ±4.00   |        2.27           |
    | `HMC_GAIN470` |    5   |     390      |   ±4.70   |        2.56           |
    | `HMC_GAIN560` |    6   |     330      |   ±5.60   |        3.03           |
    | `HMC_GAIN810` |    7   |     230      |   ±8.10   |        4.35           |

    @return Returns `0` on no error. Otherwise returns error code. Returns I2C errors from calls to
            `write_data()`, as well as:
            - \c `EC_BAD_GAIN_LEVEL` Returned if input gain level is out of range.
    */

    // Validate input
    if (gain_level > 7) {
        return EC_BAD_GAIN_LEVEL;
    }

    uint8_t rv = 0;

    // Write the data to the configuration register. On failure, return error code.
    if(rv = I2CDevice.write_data(HMC5883L_ADDR, ConfigRegisterB, gain_level << 5)) { return rv; }

    // Update gain value cache.
    gain = gain_level;

    return 0;
}

uint8_t HMC5883L::setAveragingRate(uint8_t avg_rate) {
    /** Set the magnetometer averaging rate.

    This is the number of averages composing each measurement. It sets bits 5 and 6 on
    Configuration Register A on the device.

    @param[in] avg_rate The averaging rate. The number of averages per measurement is 1<<avg_rate.

    | `avg_rate` | Value | Rate |
    | :--------- | :---: | :--: |
    | `HMC_AVG1` |   0   |  1   |
    | `HMC_AVG2` |   1   |  2   |
    | `HMC_AVG4` |   2   |  4   |
    | `HMC_AVG8` |   3   |  8   |

    @return Returns `0` on no error. Otherwise returns error code. Returns I2C errors from calls to
            `read_data()` and `write_data()`, as well as:
            - \c `EC_INVALID_NAVG` Returned if the number of averages is out of range.
    */
    
    // Validate input
    if (avg_rate > 3) {
        return EC_INVALID_NAVG;
    }

    // Get the configuration register value, then mask out bits 5 and 6.
    uint8_t configRegister = I2CDevice.read_data_byte(HMC5883L_ADDR, ConfigRegisterA);

    // Return error code on error.
    if (err_code = I2CDevice.get_error_code()) { 
        return err_code; 
    }

    configRegister &= 0x9f;         // Mask out bits 5 and 6.

    // Update register value.
    if (err_code = I2CDevice.write_data(ConfigRegisterA, avg_rate << 5 | configRegister)) {
        return err_code;
    }

    averagingRate = avg_rate;
    return 0;
}

uint8_t HMC5883L::setOutputRate(uint8_t out_rate) {

}

uint8_t HMC5883L::setMeasurementMode(uint8_t mode) {

}

uint8_t HMC5883L::setBiasMode(uint8_t mode) {

}

uint8_t HMC5883L::getGain(bool updateCache=false) {

}

uint8_t HMC5883L::getAveragingRate(bool updateCache=false) {

}

uint8_t HMC5883L::getOutputRate(bool updateCache=false) {

}

uint8_t HMC5883L::getMeasurementMode(bool updateCache=false) {

}

uint8_t HMC5883L::getBiasMode(bool updateCache=false) {

}
