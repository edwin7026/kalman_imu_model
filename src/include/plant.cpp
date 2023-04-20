// This file holds the implementation of world class

#include "plant.h"
#include <sstream>

plant::plant()
{   
    // Set position to 0
    pos << 0, 0, 0;

    // Zero out orientation
    theta << M_PI/2, M_PI/2, 0;
    
    // Zero out angular velocity
    omega << 0, 0, 0;
    
    // Set simulation time to 0
    t = 0;

    // Debug
    J << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    dt = 0.01;
}

plant::plant(const std::string& file_path, Eigen::Vector3f* romega, Eigen::Vector3f* racc)
{
    // Get all the parameters
    get_config(file_path);

    // Set outputs
    real_omega = romega;
    real_acc = racc;
    
    // Zero out position, orientation and angular velocity
    zero_out();
}

int plant::get_config(const std::string file_path)
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
        const libconfig::Setting &inertial_params = root["inertial_param"];
        const libconfig::Setting &sim_params = root["sim_param"];

        // Set simulation parameters

        // Get dt
        if (sim_params.lookupValue("delta_t", dt));
        else
            std::cout << "Couldn't read dt" << std::endl;
        
        // Get simulation length
        if (sim_params.lookupValue("sim_len", sim_len));
        else
            std::cout << "Couldn't read sim_len" << std::endl;
        
        // Set inertial parameters
        if (inertial_params.lookupValue("mass", mass));
        else
            std::cout << "Couldn't read mass" << std::endl;

        // Get the moment of inertia data
        // TODO Exception handling
        const libconfig::Setting& J_vals = inertial_params.lookup("J");
        for (unsigned i = 0; i < 3; i++)
            for (unsigned j = 0; j < 3; j++)
                J(i, j) = inertial_params.lookup("J")[3*i + j];
        
        // Store its inverse
        J_inv = J.inverse(); 

    }
    catch(const libconfig::SettingNotFoundException &nfex)
    {
        // Ignore
    }

    return(EXIT_SUCCESS);
}

void plant::gen_skew_matrix(Eigen::Vector3f* vect, Eigen::Matrix3f* skew_mat)
{
    (*skew_mat) << 0,          -(*vect)(2),      -(*vect)(1), 
                (*vect)(2),              0,      -(*vect)(0), 
               -(*vect)(1),     (*vect)(0),                0;
}

float plant::differentiator(float y2, float y1)
{
    return (y2 - y1) / dt;
}

bool plant::set_torque(Eigen::Vector3f* torq_in)
{
    torq = *torq_in;
    return next_sim_step();
}

bool plant::next_sim_step()
{   
    // Get skew symmetric matrix
    gen_skew_matrix(&omega, &omega_skew);

    // Plant dynamics
    Jomega_dot = -omega_skew * J * omega + torq;
    omega_dot = J_inv * Jomega_dot;
    
    // Integrate to get kinematic paramters
    omega += omega_dot * dt;
    theta += omega * dt;

    // TODO Translational motion

    // Wrap back orientation
    if (abs(theta(0)) > 2*M_PI)
        theta(0) = theta(0) - ((int)(theta(0) / (2*M_PI))) * 2*M_PI;
    if (abs(theta(1)) > 2*M_PI)
        theta(1) = theta(1) - ((int)(theta(1) / (2*M_PI))) * 2*M_PI;
    if (abs(theta(2)) > 2*M_PI)
        theta(2) = theta(2) - ((int)(theta(2) / (2*M_PI))) * 2*M_PI;

    // Set the interface variables
    (*real_omega) = omega;
    (*real_acc) = force / mass;
 
    // Increment simulation time
    t = t + dt;

    // Check if simulation time exceeded simulation length
    if (t > sim_len)
        return true;
    return false;
}

void plant::zero_out()
{
    t = 0;
    pos << 0, 0, 0;
    theta << 0, 0, 0;       // TODO
    omega << 0, 0, 0;
    omega_dot << 0, 0, 0;
}

void plant::print_state()
{
    std::cout //<< "Mass: " << mass << " kg\n"
        //<< "Moment of inertia:\n" << J
        //<< "\nPosition:\n" << pos << " m\n"
        << "\nOrientation:\n" << theta << " rad\n"
        << "\nTorq:\n" << torq << " N-m\n"
        //<< "\nForce:\n" << force << " N\n"
        << "\nAngular Velocity:\n" << omega << " rad/s\n"
        << "\nAngular acceleration:\n" << omega_dot << " rad/s^2\n"
        << "\nCurrent time: " << t << " s\n"
        << "\nTick granularity: " << dt << " s\n";
}