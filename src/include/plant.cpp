// This file holds the implementation of world class

#include "plant.h"

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

void plant::set_torque(Eigen::Vector3f* torq_in)
{
    torq = *torq_in;
    next_sim_step();    
}

void plant::next_sim_step()
{   
    // Get skew symmetric matrix
    gen_skew_matrix(&omega, &omega_skew);

    // Plant dynamics
    Jomega_dot = -omega_skew * J * omega + torq;
    omega_dot = J.inverse() * Jomega_dot;
    
    // Integrate to get kinematic paramters
    omega += omega_dot * dt;
    theta += omega * dt;
    
    // Increment simulation time
    t += dt;

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
