/** @file
Header file for HMC5883L three-axis magnetometer primary interface to Arduino over I2C.

This code is released under a Creative Commons Attribution 4.0 International license 
([CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/)).

@author Paul J. Ganssle
@version 0.2
@date 2015-01-14
*/

#ifndef HMC5883L_H
#define HMC5883L_H

#include <I2CDev.h>
#include <Vec3.h>
#include <Wire.h>

/** @defgroup DeviceAddrs Device addresses
@{ */
#define HMC5883L_ADDR 0x1E          /*!< The I2C address of all HMC5883L digital magnetometers */

#define ConfigRegisterA 0x00        /*!< Register address for Config Register A, which contains:
                 | Location | Description                                                    |
                 | :------: | :------------------------------------------------------------- |
                 |   5 - 6  | Number of samples averaged, see `HMC5883L::setAveragingRate()` |
                 |   2 - 4  | Data output rate, see `HMC5883L::setOutputRate()`              |
                 |   0 - 1  | Bias measurement register, see `HMC5883L::setBiasMode()`       | */

#define ConfigRegisterB 0x01        /*!< Register address for Config Register A, which contains:
                 | Location | Description                                     |
                 | :------: | :---------------------------------------------- |
                 |   5 - 7  | Gain configuration, see `HMC5883L::setGain()`   |
                 |   0 - 4  | Not used - must be cleared for proper operation | */

#define ModeRegister 0x02           /*!< Register address for the mode register, which contains:
                 | Location | Description                                                        |
                 | :------: | :----------------------------------------------------------------- |
                 |     7    | High speed I2C mode bit, see `HMC5883L::setHighSpeedI2CMode()`     |
                 |   1 - 6  | Not used - cleared by default                                      |
                 |   0 - 1  | Measurement mode select bits, see `HMC5883L::setMeasurementMode()` |
                 */

#define DataRegister 0x03           /*!< Starting address for the data registers, which are, in
                                         order: `DXRA` (MSB), `DXRB` (LSB), `DZRA` (MSB), 
                                         `DZRB` (LSB), `DYRA` (MSB), `DYRB` (LSB). */
#define StatusRegister 0x09         /*!< Register address for the status register, which contains
                                         the `LOCK` [1] and `RDY` [0]. See `getStatus()`. */

/** @}*/

/** @defgroup GeneralConstants General Constants
Constants related to device operations. 
@{ */
#define HMC_SLEEP_DELAY 7           /*!< Sleep delay in milliseconds (rounded up from 160 Hz) */
#define HMC_BIAS_XY 1160.0          /*!< Bias applied by the self-test coils along X and Y, in mG */
#define HMC_BIAS_Z 1080.0           /*!< Bias applied by the self-test coils along Z, in mG */
/** @} */

/** @defgroup DeviceSettings Device settings
Convenient definitions for the various device settings.
@{ */

/** @defgroup GainSettings Gain settings
@ingroup DeviceSettings
Convenient definitions for the gain settings. See `setGain()` for more details.
@{ */

#define HMC_GAIN088 0   /*!< Gain: 1370 LSB/G,  Range: ±0.88, Resolution:  0.73 (mG / LSB) */
#define HMC_GAIN130 1   /*!< Gain: 1090 LSB/G,  Range: ±1.30, Resolution:  0.92 (mG / LSB) */
#define HMC_GAIN190 2   /*!< Gain:  820 LSB/G,  Range: ±1.90, Resolution:  1.22 (mG / LSB) */
#define HMC_GAIN250 3   /*!< Gain:  660 LSB/G,  Range: ±2.50, Resolution:  1.52 (mG / LSB) */
#define HMC_GAIN400 4   /*!< Gain:  440 LSB/G,  Range: ±4.00, Resolution:  2.27 (mG / LSB) */
#define HMC_GAIN470 5   /*!< Gain:  390 LSB/G,  Range: ±4.70, Resolution:  2.56 (mG / LSB) */
#define HMC_GAIN560 6   /*!< Gain:  330 LSB/G,  Range: ±5.60, Resolution:  3.03 (mG / LSB) */
#define HMC_GAIN810 7   /*!< Gain:  230 LSB/G,  Range: ±8.10, Resolution:  4.35 (mG / LSB) */

/** @} */

/** @defgroup AvgSettings Averaging settings
@ingroup DeviceSettings
Convenient definitions for the averaging settings. See `setAveragingRate()` for more details. 
@{ */
#define HMC_AVG1 0  /*!< Data output is 1 average. */
#define HMC_AVG2 1  /*!< Data output is 2 average. */
#define HMC_AVG4 2  /*!< Data output is 4 average. */
#define HMC_AVG8 3  /*!< Data output is 8 average. */
/** @} */

/** @defgroup OutputRates Output rate settings
@ingroup DeviceSettings
Convenient definitions for the output rates in continuous mode. See `setOutputRate()` for details.
@{ */
#define HMC_RATE0075 0 /*!< Rate:  0.75 Hz */
#define HMC_RATE0150 1 /*!< Rate:  1.50 Hz */
#define HMC_RATE0300 2 /*!< Rate:  3.00 Hz */
#define HMC_RATE0750 3 /*!< Rate:  7.50 Hz */
#define HMC_RATE1500 4 /*!< Rate: 15.00 Hz */
#define HMC_RATE3000 5 /*!< Rate: 30.00 Hz */
#define HMC_RATE7500 6 /*!< Rate: 75.00 Hz */
/** @} */

/** @defgroup MeasurementModes Device measurement modes
@ingroup DeviceSettings
Convenient definitions for the measurement modes, see `setMeasurementMode()` for details.
@{ */
#define HMC_MeasurementContinuous 0     /*!< Continuous measurements */
#define HMC_MeasurementSingle 1         /*!< Single shot - one measurement */
#define HMC_MeasurementIdle 2           /*!< Idle mode - No measurements */
/** @} */

/** @defgroup BiasModes Measurement bias modes
@ingroup DeviceSettings
Convenient definitions for the measurement bias modes, see `setBiasMode()` for details.
@{ */
#define HMC_BIAS_NONE 0         /*!< No bias */
#define HMC_BIAS_POSITIVE 1     /*!< Positive bias */
#define HMC_BIAS_NEGATIVE 2     /*!< Negative bias */
/** @} */
/** @} */

/** @defgroup ErrorCodes Errors and warnings
Error codes and warning codes. Error codes can be retrieved from `HMC5883L::get_err_code()`.
@{ */
#define EC_BAD_GAIN_LEVEL 8                 /*!< Gain input value is out of range. */
#define EC_INVALID_NAVG 9                   /*!< Number of average is out of range. */
#define EC_INVALID_OUTRATE 10               /*!< Output rate is out of range. */
#define EC_INVALID_MEASUREMENT_MODE 11      /*!< Invalid measurement mode specified.  */
#define EC_INVALID_BIAS_MODE 12             /*!< Invalid bias mode specified. */
#define EC_INVALID_UFLOAT 13                /*!< Float specified cannot be negative. */

/** @defgroup SaturationWarningCodes Saturation warning codes
@ingroup ErrorCodes
Flags raised when various channels are read as saturated. These flags are the first 3 bits of a
byte, so if X and Y are saturated but not Z, the code will be `WC_X_SATURATED | WC_Y_SATURATED`,
etc.
@{
*/
#define WC_X_SATURATED 1    /*!< Warning: X channel is saturated. */
#define WC_Y_SATURATED 2    /*!< Warning: Y channel is saturated. */
#define WC_Z_SATURATED 4    /*!< Warning: Z channel is saturated. */
/** @} */
/** @} */

class HMC5883L {
    /** HMC5883L 3-axis digital magnetometer class object */
public:
    HMC5883L();

    uint8_t initialize(bool noConfig=false);

    Vec3<int> readRawValues(uint8_t *saturated=NULL);
    Vec3<float> readScaledValues(uint8_t *saturated=NULL);
    Vec3<float> readScaledValuesSingle(uint8_t *saturated=NULL, uint32_t max_retries=0,
                                       uint32_t delay_time=HMC_SLEEP_DELAY);
    Vec3<float> readCalibratedValues(uint8_t *saturated=NULL);
    Vec3<float> readCalibratedValuesSingle(uint8_t *saturated=NULL, uint32_t max_retries=0,
                                           float delay_time=HMC_SLEEP_DELAY);

    Vec3<float> getCalibration(bool update, uint8_t *saturated=NULL, 
                               uint32_t max_retries=0, float delay_time=HMC_SLEEP_DELAY);

    Vec3<float> runPosTest(uint8_t *saturated=NULL, uint32_t max_retries=0,
                           float delay_time=HMC_SLEEP_DELAY);
    Vec3<float> runNegTest(uint8_t *saturated=NULL, uint32_t max_retries=0,
                           float delay_time=HMC_SLEEP_DELAY);

    uint8_t getStatus(bool *isLocked, bool *isReady);

    uint8_t setGain(uint8_t gain_level);
    uint8_t setAveragingRate(uint8_t avg_rate);
    uint8_t setOutputRate(uint8_t out_rate);
    uint8_t setMeasurementMode(uint8_t mode);
    uint8_t setBiasMode(uint8_t mode);
    uint8_t setHighSpeedI2CMode(bool enabled);

    uint8_t getGain(bool updateCache=false);
    uint8_t getAveragingRate(bool updateCache=false);
    uint8_t getOutputRate(bool updateCache=false);
    uint8_t getMeasurementMode(bool updateCache=false);
    uint8_t getBiasMode(bool updateCache=false);

    uint8_t get_error_code(void);

    static const float outputRates[];  /*!< Output rates in Hz (see \ref OutputRates). */
    static const float gainRanges[];   /*!< Saturation ranges in mG. See \ref GainSettings */

private:
    I2CDev I2CDevice;                  /*!< The I2C interface device */
    Vec3<float> calibration;           /*!< The current calibration for the magnetometer */

    uint8_t gain;
    uint8_t averagingRate;
    uint8_t outputRate;
    uint8_t measurementMode;
    uint8_t biasMode;

    uint8_t err_code;

    static const float gainValues[];
};

#endif