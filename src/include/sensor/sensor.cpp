/*
 * This file constains the implementation of the functions defined
 * in sensors.h
 */

#include "sensor.h"

sensor::sensor(const std::string& file_path, Eigen::Vector3f* omega, Eigen::Vector3f* acc)
{
    // Activate all sensors
    acc_on = true;
    gyro_on = true;
    mag_on = true;

    // Connect sensor to the real world
    real_omega = omega;
    real_acc = acc;

    // Get configuration
    get_config(file_path);

    // Reset all measurements
    reset();
}

int sensor::get_config(const std::string file_path)
{
    libconfig::Config cfg;

    // Read the file. If there is an error, report and exit
    try
    {
        cfg.readFile(file_path.c_str());
    }
    catch(const libconfig::FileIOException &fioex)
    {
        std::cerr << "I/O error while reading file." << std::endl;
        return(EXIT_FAILURE);
    }

    // Get root level node
    const libconfig::Setting& root = cfg.getRoot();
    
    try
    {
        const libconfig::Setting &acc_params = root["acc_param"];
        const libconfig::Setting &gyro_params = root["gyro_param"];

        // Set parameters for accelerometer

        // Get drift
        //const libconfig::Setting& acc_drift = acc_params.lookup("drift");
        //TODO Exception handling
        for (unsigned i = 0; i < 3; i++)
            acc_drift(i) = acc_params.lookup("drift")[i];
        
        // Get simulation length
        if (acc_params.lookupValue("noise_power", acc_noise));
        else
            std::cout << "Couldn't read sim_len" << std::endl;

        // Set parameters for gyroscope

        // Get drift
        // TODO Exception handling
        for (unsigned i = 0; i < 3; i++)
            acc_drift(i) = acc_params.lookup("drift")[i];
        
        // Get simulation length
        if (acc_params.lookupValue("noise_power", acc_noise));
        else
            std::cout << "Couldn't read sim_len" << std::endl;

    }
    catch(const libconfig::SettingNotFoundException &nfex)
    {
        // Ignore
    }

    return(EXIT_SUCCESS);
}

void sensor::compute_sensor_out()
{
    // TODO

    gyro_out = *real_omega;
    acc_out = *real_acc;

}

void sensor::reset()
{
    acc_out << 0, 0, 0;
    gyro_out << 0, 0, 0;

    // TODO: magnetometer is not integrated
    mag_out << 0, 0, 0;
}

void sensor::get_acc_data(Eigen::Vector3f* data)
{   
    (*data) = acc_out;
}

void sensor::get_gyro_data(Eigen::Vector3f* data)
{
    (*data) = gyro_out;
}

void sensor::get_mag_data(Eigen::Vector3f* data)
{
    (*data) = mag_out;
}

void sensor::print_state()
{
    std::cout << "\nGyroscope output:\n" << gyro_out << " rad/s\n"
        << "\nAcceleration:\n" << acc_out << " m/s^2\n";
}