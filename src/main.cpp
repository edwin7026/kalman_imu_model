#include <iostream>
#include <thread>
#include <plant.h>

static bool is_finished = false;

void run_simulation()
{
    while(!is_finished)
    {
        plant a;
        Eigen::Vector3f tor = Eigen::Vector3f::Random() * 10;
        a.set_torque(&tor);
        a.print_state();
        std::cout << "\n***************************************************************"
                << std::endl;
    }
}

int main()
{   
    std::thread worker(run_simulation);
    
    // Wait for keyboard interrupt
    std::cin.get();

    // Set the flag to true
    is_finished = true;
    
    // Join all threads
    worker.join();

    
    std::cout << "End of Simulation" << std::endl;
}
