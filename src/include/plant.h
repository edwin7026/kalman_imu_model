/**
 * @file This header file basic kinematic data
 */

#include <Eigen/Dense>
#include <iostream>
#include <cmath>

/**
 * This class holds the dynamics and kinematics of the body
 */
class plant
{

private:

    // Inertial characteristics
    float mass;                          // Mass of the system
    Eigen::Matrix3f J;                   // Moment of inertia tensor
    
    // Spatial features
    Eigen::Vector3f pos;
    Eigen::Vector3f theta;

    // Dynamic characteristics
    Eigen::Vector3f torq;                // Torque applied to the system
    Eigen::Vector3f force;               // Force on the system
    
    Eigen::Vector3f omega;               // Angular velocity
    Eigen::Vector3f omega_dot;           // Angular acceleration

    // Helper variables
    Eigen::Matrix3f omega_skew;
    Eigen::Vector3f Jomega_dot;

    // Time
    float t;                            // Current simulation time
    float dt;                           // Step interval

     // Transformation function
    void next_sim_step();

public:
    
    // Constructor //
    plant();

    // Plant configuration functions //
    //TODO Get inertial and simualtion parameters from a configuration file

    // Math functions //
    
    /**
     * Provided the pointer to a vector vect and a matrix skew_mat, assigns 
     * a skew symmetric version of vect to skew_mat
     */
    void gen_skew_matrix(Eigen::Vector3f* vect, Eigen::Matrix3f* skew_mat);
    
    /**
     * A simple discrete differentiator
     */
    float differentiator(float y2, float y1);

    // Input functions //
    
    /**
     * Sets torque to the system and calls the next_sim_step() function
     */
    void set_torque(Eigen::Vector3f* torq_in);

    
    // Debug functions
    void print_state();
};
