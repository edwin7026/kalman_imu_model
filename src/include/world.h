/**
 * @file This header file basic kinematic data
 */

/**
 * This class holds the dynamics and kinematics of the system
 */
class world
{

private:

    // Inertial characteristics
    float mass;                 // Mass of the system
    float J[3][3];              // Moment of inertia tensor
    
    // Spatial features
    float pos[3];
    float theta[3];

    // Time
    float t;            // Current simulation time
    float dt;           // Step interval

    // Dynamic characteristics
    float torq[3];      // Torque applied to the system
    float force[3];     // Force on the system

public:
    
    // Constructor
    world();

    // Transformation functions
};
