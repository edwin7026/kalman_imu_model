#include <iostream>
#include <cmath>

#include <eigen3/Eigen/Dense>
#include <libconfig.h++>

/**
 * @class This class models an IMU sensor
 **/
class sensor
{

private: 
    // Selectively turn on/off sensors
    bool acc_on;
    bool gyro_on;
    bool mag_on;

    // Real world data
    Eigen::Vector3f* real_omega;
    Eigen::Vector3f* real_acc;
    
    // Ouput data from the sensor
    Eigen::Vector3f acc_out;
    Eigen::Vector3f gyro_out;

    // TODO yet to integerate
    Eigen::Vector3f mag_out;

    // Sensor noise attributes
    float acc_noise;
    float gyro_noise;

    // Sensor drift
    Eigen::Vector3f acc_drift;
    Eigen::Vector3f gyro_drift;

public:

    // Constructor
    sensor(const std::string&, Eigen::Vector3f*, Eigen::Vector3f*);

    // Function to get configuration file for the sensor
    int get_config(const std::string);

    /**
     * This function adds noise and drift to the sensor output
     */
    void compute_sensor_out();
    
    // Tare all components of the sensor to 0 
    void reset();
    
    /*********************************************************
     * Data access functions
     **********************************************************/ 
    
    // Assigns accelerometer data to the input pointer
    void get_acc_data(Eigen::Vector3f*);

    // Return gyroscope data to the input pointer
    void get_gyro_data(Eigen::Vector3f*);

    // Assigns magnetometer data to the input pointer
    void get_mag_data(Eigen::Vector3f*);

    // Print state of the sensor output
    void print_state();
};