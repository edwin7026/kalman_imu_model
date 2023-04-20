#include <iostream>
#include <thread>

#include <plant.h>
#include <sensor/sensor.h>

static bool is_finished = false;

void run_simulation()
{   
    // Plant-sensor interface
    Eigen::Vector3f omega;
    Eigen::Vector3f acc;

    // Set the parameters from the configuration files
    plant rocket("plant_config.cfg", &omega, &acc);
    sensor sensor_device("sensor_config.cfg", &omega, &acc);

    // Run the loop till a keyboard enter key is found or 
    // simulation time exceeded sim_len
    while(!is_finished)
    {
        Eigen::Vector3f tor = Eigen::Vector3f::Random() * 10;
        is_finished = rocket.set_torque(&tor);
        sensor_device.compute_sensor_out();
        //rocket.print_state();
        sensor_device.print_state();
        std::cout << "\n***************************************************************"
             << std::endl;
    }
}

int main()
{   
    // Spawns run_simulation function to run concurrently
    std::thread worker(run_simulation);
    
    // Wait for keyboard interrupt
    std::cin.get();

    // Set the flag to true
    is_finished = true;
    
    // Join all threads
    worker.join();

    std::cout << "End of Simulation" << std::endl;

    return 0;
}
