#include "LSM6DSOXFIFOWrapper.h"

LSM6DSOXFIFO::LSM6DSOXFIFO(TwoWire &wire, uint8_t address)
    : lsm6dsoxSensor(&wire, address)
{
    logCallback = nullptr;
    dataReadyCallback = nullptr;
}

int LSM6DSOXFIFO::initialize(void)
{
    // Initialize sensors
    lsm6dsoxSensor.begin();
    if (lsm6dsoxSensor.Enable_G() == LSM6DSOX_OK && lsm6dsoxSensor.Enable_X() == LSM6DSOX_OK)
    {
        log("Success in enabling accelerometer and gyroscope\n");
    }
    else
    {
        log("Error in enabling accelerometer and gyroscope\n");
        return false;
    }

    // Check device id
    uint8_t device_id;
    lsm6dsoxSensor.ReadID(&device_id);
    if (device_id != LSM6DSOX_ID)
    {
        log("Wrong ID for LSM6DSOX sensor. Check device is plugged\n");
        return false;
    }
    else
    {
        log("Success checking ID for LSM6DSOX sensor\n");
    }

    // Set accelerometer scale. Available values are: 2, 4, 8, 16 G
    lsm6dsoxSensor.Set_X_FS(IMU_ACCELEROMETER_SCALE);
    // Set gyroscope scale. Available values are: 125, 250, 500, 1000, 2000 dps
    lsm6dsoxSensor.Set_G_FS(IMU_GYROSCOPE_SCALE);

    // Set accelerometer Output Data Rate. Available values are: 1.6, 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz
    lsm6dsoxSensor.Set_X_ODR(IMU_SAMPLING_RATE);
    // Set gyroscope Output Data Rate. Available values are 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz
    lsm6dsoxSensor.Set_G_ODR(IMU_SAMPLING_RATE);

    // Set FIFO Batch Data Rate for accelerometer and gyroscope. Available values are: 0, 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz
    lsm6dsoxSensor.Set_FIFO_X_BDR(IMU_SAMPLING_RATE);
    lsm6dsoxSensor.Set_FIFO_G_BDR(IMU_SAMPLING_RATE);

    /** Set FIFO operation mode. Available values are:
     * LSM6DSOX_BYPASS_MODE: FIFO is not used, the buffer content is cleared
     * LSM6DSOX_FIFO_MODE: bufer continues filling until it becomes full. Then it stops collecting data.
     * LSM6DSOX_STREAM_MODE: continuous mode. Older data are replaced by the new data.
     * LSM6DSOX_STREAM_TO_FIFO_MODE: FIFO buffer starts operating in Continuous mode and switches to FIFO mode when an event condition occurs.
     * LSM6DSOX_BYPASS_TO_STREAM_MODE: FIFO buffer starts operating in Bypass mode and switches to Continuous mode when an event condition occurs.
     * */
    lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_BYPASS_MODE); // Flush any previous value in FIFO before start
    lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_STREAM_MODE); // Start batching in continous mode

    // Set FIFO watermark level. Can be used to check when the number of samples in buffer reaches this defined threshold level.
    lsm6dsoxSensor.Set_FIFO_Watermark_Level(IMU_FIFO_WATERMARK_LEVEL);

    return true;
}

void LSM6DSOXFIFO::update(void)
{
    // Check if FIFO threshold level was reached.
    uint8_t fifo_watermark_threshold_reached = 0; // FIFO watermark status
    lsm6dsoxSensor.Get_FIFO_Watermark_Status(&fifo_watermark_threshold_reached);
    if (fifo_watermark_threshold_reached)
    {
        // Fetch data from FIFO
        for (uint16_t i = 0; i < IMU_FIFO_WATERMARK_LEVEL; i++)
            readFIFObuffer();
    }

    // Check if FIFO is full.
    uint8_t fifo_full = 0; // FIFO full status
    lsm6dsoxSensor.Get_FIFO_Full_Status(&fifo_full);

    if (fifo_full)
    {
        log("-- FIFO is full! Consider reducing Watermark Level or Buffer Data Rate.\n");
        log("Flushing data from FIFO.\n");
        lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_BYPASS_MODE); // Flush FIFO data
        lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_STREAM_MODE); // Continue batching
    }
}

int LSM6DSOXFIFO::readFIFObuffer(void)
{
    int ret_val = 0;
    uint8_t fifo_tag;                       // FIFO data sensor identifier
    lsm6dsoxSensor.Get_FIFO_Tag(&fifo_tag); // Get data identifier

    switch (fifo_tag)
    {
    [[likely]] case IMU_FIFO_TAG_GYROSCOPE:
        // Get gyroscope data
        if (data.rotation_data_ready)
            log("Overwriting rotation data for a more recent ones.\n");
        lsm6dsoxSensor.Get_FIFO_G_Axes(vector3intSerialize(&data.rotation_data));
        data.rotation_data_ready = true;
        ret_val = IMU_FIFO_TAG_GYROSCOPE;
        break;
    [[likely]] case IMU_FIFO_TAG_ACCELEROMETER:
        // Get accelerometer data
        if (data.acceleration_data_ready)
            log("Overwriting acceleration data for a more recent ones.\n");
        lsm6dsoxSensor.Get_FIFO_X_Axes(vector3intSerialize(&data.acceleration_data));
        data.acceleration_data_ready = true;
        ret_val = IMU_FIFO_TAG_ACCELEROMETER;
        break;
    [[unlikely]] default:
        // Ignore everything else.
        log("Discarding FIFO data TAG ID %02d.\n", fifo_tag);
        ret_val = 0;
        break;
    }

    if (isDataReady() && dataReadyCallback)
    {
        dataReadyCallback(&data);
        data.flags = false;
    }

    return ret_val;
}

void LSM6DSOXFIFO::print(void)
{
    static uint32_t last_sample_millis = millis();
    uint32_t delta_millis = millis() - last_sample_millis;

    log("[IMU] [%4lu ms], ", delta_millis);
    if (data.acceleration_data_ready)
        log("Acc: [%6.3f, %6.3f, %6.3f] G, ", data.acceleration_data.X / 1000.0, data.acceleration_data.Y / 1000.0, data.acceleration_data.Z / 1000.0); // Acceleration
    if (data.rotation_data_ready)
        log("Gyro: [%8.2f, %8.2f, %8.2f] DPS", data.rotation_data.X / 1000.0, data.rotation_data.Y / 1000.0, data.rotation_data.Z / 1000.0); // Angular Velocity
    log("%s", "\n");
    last_sample_millis = millis();
}

void LSM6DSOXFIFO::registerLoggingCallback(log_callback_t callback)
{
    logCallback = callback;
}

void LSM6DSOXFIFO::registerDataReadyCallback(data_ready_callback_t callback)
{
    dataReadyCallback = callback;
}

int32_t *LSM6DSOXFIFO::vector3intSerialize(vector3int_t *vector) const
{
    return reinterpret_cast<int32_t *>(vector);
}

int LSM6DSOXFIFO::log(const char *format, ...) const
{
    if (!logCallback)
        return 0;
    char buffer[64];
    int ret_val;
    va_list va;
    va_start(va, format);
    ret_val = vsnprintf(buffer, sizeof(buffer), format, va);
    va_end(va);
    logCallback(buffer);
    return ret_val;
}

bool LSM6DSOXFIFO::isDataReady(void)
{
    return (data.acceleration_data_ready && data.rotation_data_ready);
}
