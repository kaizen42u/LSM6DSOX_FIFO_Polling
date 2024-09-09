#pragma once

#include <Arduino.h>

#include <functional>
#include <stdarg.h>

#include "LSM6DSOXSensor.h" // This library can be installed by searching `STM32duino LSM6DSOX` in the Library Manager
#include "LSM6DSOXConfig.h"

class LSM6DSOXFIFO
{
public:
    typedef struct vector3int
    {
        int32_t X;
        int32_t Y;
        int32_t Z;
    } vector3int_t;

    typedef struct imu_data
    {
        vector3int_t acceleration_data; // X, Y, Z accelerometer values in mG
        vector3int_t rotation_data;     // X, Y, Z gyroscope values in mDPS (angular velocity)
        union
        {
            struct
            {
                uint8_t acceleration_data_ready : 1; // Accelerometer values has been populated
                uint8_t rotation_data_ready : 1;     // Gyroscope values has been populated
            };
            uint8_t flags;
        };
    } imu_data_t;

    typedef std::function<int(const char *)> log_callback_t;
    typedef std::function<void(imu_data_t *)> data_ready_callback_t;

    // Constructor
    LSM6DSOXFIFO(TwoWire &wire, uint8_t address);

    // Initialize and configure IMU with given parameters.
    // The sensor will be running under FIFO buffer mode.
    //! Blocks if the sensor can not be initialize.
    int initialize(void);

    // Update sensor data
    void update(void);

    // Print sensor data
    void print(void);

    // Register logging callback
    void registerLoggingCallback(log_callback_t callback);

    // Register data ready callback
    void registerDataReadyCallback(data_ready_callback_t callback);

private:
    imu_data_t data;

    // Declare lsm6dsoxSensor as a member of the class
    LSM6DSOXSensor lsm6dsoxSensor;

    log_callback_t logCallback;
    data_ready_callback_t dataReadyCallback;

    // Converts vector3int_t into int32_t byte array for use in LSM6DSOX library code.
    int32_t *vector3intSerialize(vector3int_t *vector) const;

    // Log messages
    int log(const char *format, ...) const;

    // Check if data is ready
    bool isDataReady(void);

    // Tries to load IMU data from FIFO buffer
    // Returns `FIFO Tag ID` if success, `0` otherwise.
    int readFIFObuffer(void);
};
