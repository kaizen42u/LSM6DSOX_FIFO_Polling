#pragma once

#define IMU_SAMPLING_RATE 12.5       // Sampling rate. Options are: 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333 and 6667 Hz.
#define IMU_FIFO_WATERMARK_LEVEL 2   // Watermark threshold level. Max samples in this FIFO configuration is 512 (accel and gyro only).
#define IMU_ACCELEROMETER_SCALE 4    // Accelerometer scale. Available values are: 2, 4, 8, 16 G
#define IMU_GYROSCOPE_SCALE 500      // Gyroscope scale. Available values are: 125, 250, 500, 1000, 2000 DGS
#define IMU_FIFO_TAG_GYROSCOPE 1     // FIFO tag for indicating this FIFO stores gyroscope data
#define IMU_FIFO_TAG_ACCELEROMETER 2 // FIFO tag for indicating this FIFO stores accelerometer data