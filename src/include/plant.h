/**
 * @file This header file describes the kinamatics and dynamics of the plant
 * @author Edwin Joy <edwin7026@gmail.com>
 */

#include <eigen3/Eigen/Dense>
#include <libconfig.h++>

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <fstream>

/**
 * This class holds the dynamics and kinematics of the body
 */
class plant
{

private:
    // Inertial characteristics
    float mass;                          // Mass of the system
    Eigen::Matrix3f J;                   // Moment of inertia tensor
    
    Eigen::Vector3f g;                   // Acceleration due to gravity
    
    // Spatial features
    Eigen::Vector3f pos;
    Eigen::Vector3f theta;

    // Dynamic characteristics
    Eigen::Vector3f torq;                // Torque applied to the system
    Eigen::Vector3f force;               // Force on the system
    Eigen::Vector3f acc;                 // Acceleration of the system
    Eigen::Vector3f vel;                 // Velocity of the system
    Eigen::Vector3f omega;               // Angular velocity
    Eigen::Vector3f omega_dot;           // Angular acceleration

    // Helper variables
    Eigen::Matrix3f J_inv;
    Eigen::Matrix3f omega_skew;
    Eigen::Vector3f Jomega_dot;
    Eigen::Vector3f dir_vect;

    // Plant-sensor interface variables
    Eigen::Vector3f* real_omega;
    Eigen::Vector3f* real_acc;

    // Time
    float t;                            // Current simulation time
    float dt;                           // Step interval
    float sim_len;                      // Length of simulation

    /**
     * This function computes the next step
     * It returns a boolean value with the following meaning:
     *      true: if simulation time is greater than sim_len
     *      false: if simulation time is less than sim_len
    */
    bool next_sim_step();

public:
    
    // Constructor //
    plant();                    // use this for debug puposes
    
    plant(const std::string&, Eigen::Vector3f*, Eigen::Vector3f*);

    // Plant configuration functions //
    
    /**
     * This function sets the parameters of the plant from plant_config.cfg
     */
    int get_config(const std::string);

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
    bool set_torque(Eigen::Vector3f* torq_in);
 

    /**
     * Zero out all kinematic parameters
     */
    void zero_out();

    // Debug functions
    void print_state();
};
