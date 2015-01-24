/** @file
Class file for HMC5883L three-axis magnetometer primary interface to Arduino over I2C.

This code is released under a Creative Commons Attribution 4.0 International license 
([CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/)).

@author Paul J. Ganssle
@version 0.2
@date 2015-01-14
*/

#include <HMC5883L.h>
#include <I2CDev.h>
#include <Vec3.h>
#include <unistd.h>

HMC5883L::HMC5883L() {
    /**  Constructor for HMC5883L compass / magnetometer class. */
    I2CDevice = I2CDev(HMC5883L_ADDR);
}

const float HMC5883L::outputRates[] = {0.75, 1.50, 3.00, 7.50, 15.00, 30.00, 75.00};
const float HMC5883L::gainRanges[] = {880, 1300, 1900, 2500, 4000, 4700, 5600, 8100};
const float HMC5883L::gainValues[] = {0.73, 0.92, 1.22, 1.52, 2.27, 2.56, 3.03, 4.35};

uint8_t HMC5883L::initialize(bool noConfig) {
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

    // Initialize the calibration to (1.0, 1.0, 1.0)
    calibration = Vec3<float>(1.0, 1.0, 1.0);

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

Vec3<int> HMC5883L::readRawValues(uint8_t *saturated) {
    /** Read the raw values from the device
    
    Values on the HMC5883L are stored in the 6 data registers, in two's complement form, with
    each axis stored as two 8-bit integers (big-endian). This reads the raw integer values from
    the registers. For data under- and overflows, the registers are set to -4096 - this is detected
    and indicated with the output parameter `saturated`.

    @param[out] saturated A warning code with flags `WC_X_SATURATED`, `WC_Y_SATURATED` and
                          `WC_Z_SATURATED` indicating whether or not any of the channels has data
                          under- or overflow.

    @return Returns an integer 3-vector (x, y, z), or (0, 0, 0) on 
    */
    
    // Read the data from all three axes (two's complement)
    uint8_t *regValue = I2CDevice.read_data(DataRegister, 6);

    if (err_code = I2CDevice.get_err_code()) {
        return Vec3<int>(0, 0, 0);
    }

    int16_t x = regValue[0] << 8 | regValue[1];     // First two bytes are x.
    int16_t y = regValue[4] << 8 | regValue[5];     // Bytes 4 and 5 are y.
    int16_t z = regValue[2] << 8 | regValue[3];     // Bytes 2 and 3 are z.

    // Set the appropriate warning flags if the sensor is saturated.
    if (saturated != NULL) {
        *saturated = 0;
        if (x == -4096) { *saturated |= WC_X_SATURATED; }
        if (y == -4096) { *saturated |= WC_Y_SATURATED; }
        if (z == -4096) { *saturated |= WC_Z_SATURATED; }
    }

    return Vec3<int>(x, y, z);
}

Vec3<float>  HMC5883L::readScaledValues(uint8_t *saturated) {
    /** Read the field vector and return the value in milliGauss.

    Scales the integers returned by `readRawValues()` by the appropriate gain value determined by
    the value set by `setGain()`.

    @param[out] *saturated Warning flags in case any of the channels are saturated. Pass `NULL` if
                           you don't want to read these out. Default value is `NULL`.

    @return Returns a `Vec3<float>` containing the scaled values for the x, y and z channels or
            (0, 0, 0) on error.
    */

    Vec3<int> rawValues = readRawValues(saturated);

    if (err_code) {
        return Vec3<float>(0.0, 0.0, 0.0);
    }

    Vec3<float> rv = Vec3<float>(rawValues.x, rawValues.y, rawValues.z);

    return rv * gainValues[gain];
}

Vec3<float> HMC5883L::readScaledValuesSingle(uint8_t *saturated, uint32_t max_retries, 
                                             float delay_time) {
    /** Wrapper for `readScaledValues()` which makes a single measurement

    The device is put into single measurement mode, then wait `delay_time` (in milliseconds), 
    a single `readScaledValues()` measurement is made, then the measurement mode is restored to the
    initial mode.
    
    @param[out] *saturated Warning flags in case any of the channels are saturated. Pass `NULL` if
                           you don't want to read these out. Default value is `NULL`.
    @param[in] max_retries The maximum number of times to try to read the measurement. Pass 0 if
                           you don't want to limit the number of retries. Default is 0.
    @param[in] delay_time  Time to delay before checking whether or not data is ready to be read
                           from the device (also the repetition delay between checks for whether
                           data is ready), in milliseconds. The default is `HMC_SLEEP_DELAY`, which
                           is 7 ms (6.25 ms = 160 Hz, the maximum data output rate of the device, 
                           rounded up). Any non-negative value is valid.

    @return Returns a `Vec3<float>` containing the scaled values for the x, y and z channels or
            (0, 0, 0) on error. In addition to errors returned from `I2CDevice` calls, this
            returns `EC_INVALID_UFLOAT` if a negative `delay_time` is passed.
    */

    Vec3<float> zv = Vec3<float>(0.0, 0.0, 0.0);    // Returned on error
    if (delay_time < 0.0) {
        err_code = EC_INVALID_UFLOAT;
        return zv;
    }

    uint8_t mode = getMeasurementMode();

    if (err_code = setMeasurementMode(HMC_MeasurementSingle)) {
        return zv;
    }

    uint32_t retries = 0;
    bool locked, ready;
    while (max_retries && retries++ < max_retries) {
        if (err_code = getStatus(&locked, &ready)) {
            return zv;
        }

        if (!locked && ready) {
            break;
        }

        usleep(delay_time*1e3);        // Convert milliseconds to microseconds
    }

    Vec3<float> rv = readScaledValues(saturated);

    // Whether or not there's an error, try to restore the old measurement mode if possible
    uint8_t old_ec = err_code;
    if (!(err_code = setMeasurementMode(mode))) {
        err_code = old_ec;          // In case the scaledValues failed but the mode change didn't.
    }

    if (err_code) {
        return zv;
    }

    return rv;
}

Vec3<float> HMC5883L::readCalibratedValues(uint8_t *saturated) {
    /** Return the field vector, scaled by the calibration, in milliGauss.

    Makes a call to `readScaledValues()`, then scales the results by the calibration. By default,
    the calibration is (1.0, 1.0, 1.0). Make a call to `getCalibration(true)` to initialize the
    calibration. 

    @param[out] *saturated Warning flags in case any of the channels are saturated. Pass `NULL` if
                           you don't want to read these out. Default value is `NULL`.
    
    @return Returns the value of `readScaledValues()`, scaled by the calibration, in mG. On error,
            returns (0, 0, 0) and sets the error code.
    */

    // No need to check for error code - scaledValues returns a zero vector on error
    return readScaledValues(saturated) * calibration;
}

Vec3<float> HMC5883L::readCalibratedValuesSingle(uint8_t *saturated, uint32_t max_retries,
                                                 float delay_time) {
    /** Return the field vector, scaled by the calibration, in milliGauss.

    Makes a single call to `readScaledValuesSingle()`, then scales the results by the calibration
    stored in the `HMC5883L` object. By default, the calibration is `(1.0, 1.0, 1.0)`. Make a call
    to `getCalibration(true)` to initialize the calibration.
   
    @param[out] *saturated Warning flags in case any of the channels are saturated. Pass `NULL` if
                           you don't want to read these out. Default value is `NULL`.
    @param[in] max_retries The maximum number of times to try to read the measurement. Pass 0 if
                           you don't want to limit the number of retries. Default is 0.
    @param[in] delay_time  Time to delay before checking whether or not data is ready to be read
                           from the device (also the repetition delay between checks for whether
                           data is ready), in milliseconds. The default is `HMC_SLEEP_DELAY`, which
                           is 7 ms (6.25 ms = 160 Hz, the maximum data output rate of the device, 
                           rounded up). Any non-negative value is valid.

    @return Returns the value of `readScaledValuesSingle()`, scaled by the calibration, in mG. On
            error, returns (0, 0, 0) and sets the error code.
    */

    // No need to check for error code - scaledValuesSingle returns a zero vector on error
    return readScaledValuesSingle(saturated, max_retries, delay_time) * calibration;
}

Vec3<float> HMC5883L::getCalibration(bool update, uint8_t *saturated,
                                     uint32_t max_retries, float delay_time) {
    /** Runs a positive and negative bias test and sets the calibration from the average

    Runs `runPosTest()`, then `runNegTest()` and averages the values, and sets the calibration.

    @param[in] update If evaluates to true, run the calibration and update the cache. Otherwise
                      just returns the cached value.
    @param[out] *saturated Warning flags in case any of the channels are saturated. Pass `NULL` if
                           you don't want to read these out. Default value is `NULL`.
    @param[in] max_retries The maximum number of times to try to read the measurement. Pass 0 if
                           you don't want to limit the number of retries. Default is 0.
    @param[in] delay_time  Time to delay before checking whether or not data is ready to be read
                           from the device (also the repetition delay between checks for whether
                           data is ready), in milliseconds. The default is `HMC_SLEEP_DELAY`, which
                           is 7 ms (6.25 ms = 160 Hz, the maximum data output rate of the device, 
                           rounded up). Any non-negative value is valid.

    @return Returns the new calibration value. On error, returns (0, 0, 0) and sets `err_code` to
            the error.
    */

    if (update) {
        Vec3<float> zero_vec = Vec3<float>(0.0, 0.0, 0.0);      // Returned on error

        Vec3<float> pos_test = runPosTest(saturated, max_retries, delay_time);
        if (err_code) {
            return zero_vec;
        }

        Vec3<float> neg_test = runNegTest(saturated, max_retries, delay_time);
        if (err_code) {
            return zero_vec;
        }

        // Update the calibration
        calibration = (pos_test+neg_test)/2.0;
        calibration.x /= HMC_BIAS_XY;
        calibration.y /= HMC_BIAS_XY;
        calibration.z /= HMC_BIAS_Z;
    }

    return calibration;
}

Vec3<float> HMC5883L::runPosTest(uint8_t *saturated, uint32_t max_retries, float delay_time) {
    /** Runs the positive bias self-test

    Sets the bias mode to `HMC_BIAS_POSITIVE`, makes a measurement, then returns the bias mode to
    `HMC_BIAS_NONE` and returns the value of the measurement.

    @return Returns the value of a positive-biased measurement. On error, returns (0, 0, 0) and
            sets `err_code` to the error.
    */

    Vec3<float> zero_vec = Vec3<float>(0.0, 0.0, 0.0);      // Returned on error

    if (err_code = setBiasMode(HMC_BIAS_POSITIVE)) {
        return zero_vec;
    }

    Vec3<float> rv = readScaledValuesSingle(saturated, max_retries, delay_time);

    // Even if there's an error reading the scaled values, try to restore the bias mode
    uint8_t old_ec = err_code;
    if (!(err_code = setBiasMode(HMC_BIAS_NONE))) {
        err_code = old_ec;
    }

    if (err_code) {
        return zero_vec;
    }

    return rv;
}

Vec3<float> HMC5883L::runNegTest(uint8_t *saturated, uint32_t max_retries, uint32_t delay_time) {
    /** Runs the negative bias self-test

    Sets the bias mode to `HMC_BIAS_NEGATIVE`, makes a measurement, then returns the bias mode to
    `HMC_BIAS_NONE` and returns the value of the measurement.

    @return Returns the value of a negative-biased measurement. On error, returns (0, 0, 0) and
            sets `err_code` to the error.
    */

    Vec3<float> zero_vec = Vec3<float>(0.0, 0.0, 0.0);

    if (err_code = setBiasMode(HMC_BIAS_NEGATIVE)) {
        return zero_vec;
    }

    Vec3<float> rv = readScaledValuesSingle(saturated, max_retries, delay_time);

    // Even if there's an error reading the scaled values, try to restore the bias mode
    uint8_t old_ec = err_code;
    if (!(err_code = setBiasMode(HMC_BIAS_NONE))) {
        err_code = old_ec;
    }

    if (err_code) {
        return zero_vec;
    }

    return rv;
}

uint8_t  HMC5883L::getStatus(bool *isLocked, bool *isReady) {
    /** Read the status register
    
    @param[out] isLocked Whether or not the status LOCK bit is set.
    @param[out] isReady Whether or not the status RDY bit is set
    
    @return Returns the value of the status register, or a value >= 4 on error (no valid status
            register values are > 3). On error, `err_code` is also set.
    */
    
    // Read the status register and mask out the bottom two bits.
    uint8_t regValue = I2CDevice.read_data_byte(StatusRegister) & 0x3;
    if (err_code = I2CDevice.get_err_code()) {
        return 4;
    }

    // Return the individual bits
    *isLocked = regValue & 0b10;        // Lock bit
    *isReady = regValue & 0b01;         // Ready bit
    return regValue;
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
    if(rv = I2CDevice.write_data(ConfigRegisterB, gain_level << 5)) { return rv; }

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
    uint8_t configRegister = I2CDevice.read_data_byte(ConfigRegisterA) & 0x9f;
    if (err_code = I2CDevice.get_err_code()) { 
        return err_code; 
    }

    // Update register value.
    if (err_code = I2CDevice.write_data(ConfigRegisterA, (avg_rate << 5) | configRegister)) {
        return err_code;
    }

    // Update cache
    averagingRate = avg_rate;
    return 0;
}

uint8_t HMC5883L::setOutputRate(uint8_t out_rate) {
    /** Sets the data output rate in continuous output mode.
    
    Data output rate in continuous mode. Value is set in bits 2-4 of configuration register A.

    @param[in] out_rate The data output rate. Valid values are 0-6:
    
    | `out_rate`     | Value | Rate (Hz) |
    | :------------- | :---: | :-------: |
    | `HMC_RATE0075` |   0   |    0.75   |
    | `HMC_RATE0150` |   1   |    1.50   |
    | `HMC_RATE0300` |   2   |    3.00   |
    | `HMC_RATE0750` |   3   |    7.50   |
    | `HMC_RATE1500` |   4   |   15.00   |
    | `HMC_RATE3000` |   5   |   30.00   |
    | `HMC_RATE7500` |   6   |   75.00   |

    @return Returns `0` on no error. Returns I2C errors from calls to `I2CDev.read_data()` and
            `I2CDev.write_data()`, as well as:
            - \c `EC_INVALID_OUTRATE` Returned if the output rate is out of range.
    */

    // Validate input
    if (out_rate > 6) {
        return EC_INVALID_OUTRATE;
    }

    // Get the configuration register value, then mask out bits 2-4.
    uint8_t configRegister = I2CDevice.read_data_byte(ConfigRegisterA) & 0xe3;
    if (err_code = I2CDevice.get_err_code()) {
        return err_code;
    }

    // Update register value
    if (err_code = I2CDevice.write_data(ConfigRegisterA, (out_rate << 2) | configRegister)) {
        return err_code;
    }

    // Update cache
    outputRate = out_rate;
    return 0;
}

uint8_t HMC5883L::setMeasurementMode(uint8_t mode) {
    /** Sets the measurement mode (Continuous, Single or Idle).
    
    Measurement mode is the first two bits of the mode register. In continuous mode, data is output
    at the rate set by `setOutputRate`. In single measurement mode, a single measurement
    is made, the RDY pin is set high and mode is returned to idle. In single measurement mode,
    the approximate maximum data rate is 160 Hz. Idle is self-explanatory.

    @param[in] mode The measurement mode, `HMC_MeasurementContinuous` [0], 
                    `HMC_MeasurementSingle` [1], `HMC_MeasurementIdle` [2].
    
    @return Returns `0` on no error. Returns I2C errors from calls to `I2CDev.read_data()` and
            `I2CDev.write_data()`, as well as:
            - \c `EC_INVALID_MEASUREMENT_MODE` Returned if the measurement mode is out of range. 
    */

    if (mode > 2) {
        return EC_INVALID_MEASUREMENT_MODE;
    }

    // Get the configuration register, then mask out all but bit 7 (HS0 register)
    uint8_t modeRegister = I2CDevice.read_data_byte(ModeRegister) & 0x80;
    if (err_code = I2CDevice.get_err_code()) {
        return err_code;
    }

    // Update register value
    if (err_code = I2CDevice.write_data(ModeRegister, mode | modeRegister)) {
        return err_code;
    }

    // Update cache
    measurementMode = mode;
    return 0;
}

uint8_t HMC5883L::setBiasMode(uint8_t mode) {
    /** Sets the measurement bias mode (negative, positive or none).

    The HMC5883L has a self-test mode which applies either a negative or positive bias field along
    all three channels; the mode is set in the bottom two bits of the Configuration Register A.
    The applied bias fields along all three axes are:

    | Axis | Bias-on field (mG) |
    | :--: | :----------------- |
    |   X  |  ±1160             |
    |   Y  |  ±1160             |
    |   Z  |  ±1080             |

    In the negative and positive bias modes, each "measurement" consists of two measurements, a
    measurement with the bias field applied and one without, and the device returns the difference.

    @param[in] mode Valid bias modes are `HMC_BIAS_NONE` [0], `HMC_BIAS_POSITIVE` [1] and
                    `HMC_BIAS_NEGATIVE` [2].

    @return Returns `0` on no error. Returns I2C errors from calls to `I2CDev.read_data()` and
            `I2CDev.write_data()`
    */
    if (mode > 2) {
        return EC_INVALID_BIAS_MODE;
    }

    // Get the configuration register, then mask out the bottom two bits.
    uint8_t configRegister = I2CDevice.read_data_byte(ConfigRegisterA) & 0xfc;
    if (err_code = I2CDevice.get_err_code()) {
        return err_code;
    }

    // Update the register value
    if (err_code = I2CDevice.write_data(ConfigRegisterA, mode | configRegister)) {
        return err_code;
    }

    // Update cache
    biasMode = mode;
    return 0;
}

uint8_t HMC5883L::setHighSpeedI2CMode(bool enabled) {
    /** Enable or disable High Speed I2C (3400 kHz)

    @return Returns `0` on no error. returns I2C errors from calls to `I2CDev.read_data()` and
            `I2CDev.write_data()`. 
    */

    // Get the configuration register and mask out bit 7
    uint8_t modeRegister = I2CDevice.read_data_byte(ModeRegister) & 0x80;

    if (err_code = I2CDevice.get_err_code()) {
        return err_code;
    }

    // Update the register value
    err_code = I2CDevice.write_data(ModeRegister, modeRegister & (enabled?0x80:0x00));

    return err_code;
}

uint8_t HMC5883L::getGain(bool updateCache) {
    /** Retrieve the gain value

    Retrieves the gain value level that was set from `setGain()`.

    @param[in] updateCache Boolean value, default `false`. If it evaluates as true, the gain level
                           is retrieved from the device and cached in a private variable. Otherwise
                           the value is retrieved from the cache.

    @return Returns `0` on no error. Returns I2C errors from call to `I2CDev.read_data()` if
            `updateCache` is `true`. Otherwise no errors are returned.
    */
    if (updateCache) {
        uint8_t regValue = I2CDevice.read_data_byte(ConfigRegisterB);
        if (err_code = I2CDevice.get_err_code()) {
            return err_code;
        }

        gain = regValue >> 5;
    }

    return gain;
}

uint8_t HMC5883L::getAveragingRate(bool updateCache) {
    /** Retrieve the averaging rate

    Retrieves the averaging rate that was set from `setAveragingRate()`.

    @param[in] updateCache Boolean value, default `false`. If it evaluates as true, the gain level
                           is retrieved from the device and cached in a private variable. Otherwise
                           the value is retrieved from the cache.

    @return Returns `0` on no error. Returns I2C errors from call to `I2CDev.read_data()` if
            `updateCache` is `true`. Otherwise no errors are returned.
    */

    if (updateCache) {
        uint8_t regValue = I2CDevice.read_data_byte(ConfigRegisterA);
        if (err_code = I2CDevice.get_err_code()) {
            return err_code;
        }

        regValue &= 0x60;       // Mask out all but bits 5 & 6
        averagingRate = regValue >> 5;
    }

    return averagingRate;
}

uint8_t HMC5883L::getOutputRate(bool updateCache) {
    /** Retrieve the data output rate in Continuous mode

    Retrieves the data output rate that was set from `setOutputRate()`.

    @param[in] updateCache Boolean value, default `false`. If it evaluates as true, the gain level
                           is retrieved from the device and cached in a private variable. Otherwise
                           the value is retrieved from the cache.

    @return Returns `0` on no error. Returns I2C errors from call to `I2CDev.read_data()` if
            `updateCache` is `true`. Otherwise no errors are returned.
    */

    if (updateCache) {
        uint8_t regValue = I2CDevice.read_data_byte(ConfigRegisterA);
        if (err_code = I2CDevice.get_err_code()) {
            return err_code;
        }

        regValue &= 0x1c;               // Mask out everything but bits 2-4.
        outputRate = regValue >> 2;
    }

    return outputRate;
}

uint8_t HMC5883L::getMeasurementMode(bool updateCache) {
    /** Retrieve the device's measurement mode
    
    Retrieves the measurement mode that was set from `setMeasurementMode()`. If the cached mode is
    `HMC_MeasurementSingle`, the cache is always updated, irrespective of the value of
    `updateCache`, since the mode is set to `HMC_MeasurementIdle` when data is ready.

    @param[in] updateCache Boolean value, default `false`. If it evaluates as true, the gain level
                           is retrieved from the device and cached in a private variable. Otherwise
                           the value is retrieved from the cache.

    @return Returns `0` on no error. Returns I2C errors from call to `I2CDev.read_data()` if
            `updateCache` is `true`. Otherwise no errors are returned.
    */

    if (updateCache || measurementMode == HMC_MeasurementSingle) {
        uint8_t regValue = I2CDevice.read_data_byte(ModeRegister);

        regValue &= 0x3;                // Mask out all but bits 0-1.
        measurementMode = regValue;
    }

    return measurementMode;
}

uint8_t HMC5883L::getBiasMode(bool updateCache) {
    /** Retrieve the bias mode setting from the device

    Retrieves the bias mode that was set from `setBiasMode()`.

    @param[in] updateCache Boolean value, default `false`. If it evaluates as true, the gain level
                           is retrieved from the device and cached in a private variable. Otherwise
                           the value is retrieved from the cache.

    @return Returns `0` on no error. Returns I2C errors from call to `I2CDev.read_data()` if
            `updateCache` is `true`. Otherwise no errors are returned.
    */

    if (updateCache) {
        uint8_t regValue = I2CDevice.read_data_byte(ConfigRegisterA);

        regValue &= 0x3;
        biasMode = regValue;
    }

    return biasMode;
}