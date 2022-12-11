#include <iostream>

/**
 * @class This class models an IMU sensor
 **/
class sensor
{

private:

    // Real Postion and orientation
    float pos[3];
    float theta[3];

    // Analog data
    float acc[3];   // Accelerometer real data
    float gyro[3];  // Gyroscope real data
    float mag[3];   // Magnetometer real data
    
    // Selectively turn on/off sensors
    bool acc_on;
    bool gyro_on;
    bool mag_on;
    
    // Digital data
    uint8_t acc_out[3];
    uint8_t gyro_out[3];
    uint8_t mag_out[3];

    // Time
    float t;
    float tick_interval;

    // Sensor noise attributes

public:

    // Constructor
    sensor();
    
    // Tare all components of the sensor to 0 
    void reset();
    
    /*******************************************************//**
     * Data access functions
     **********************************************************/ 
    
    // Assigns accelerometer data to the input pointer
    void get_acc_data(float*);

    // Return gyroscope data to the input pointer
    void get_gyro_data(float*);

    // Assigns magnetometer data to the input pointer
    void get_mag_data(float*);
};
