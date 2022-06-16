/**
 *	\file include/dynamics.h
 *	\author Mike Timmerman
 *	\version 4.1
 *	\date 2022
 */

#pragma once

#include <Eigen/Dense>              // #include module
using namespace Eigen;              // using namespace of module

class dynamics
{
    //
	// PUBLIC MEMBER FUNCTIONS:
	//
    public:
        /** Default onstructor
         */
        dynamics();

        /** Constructor which takes the number of states and outputs, initial state and time and sampling time
         * 
         * @param[in] _nx           Number of states
         * @param[in] _nu           Number of inputs
         * @param[in] _ny           Number of outputs
         * @param[in] _initState    System initial state
         * @param[in] _samplingTime Sampling time
         * @param[in] _initTime     Initial time
         */
        dynamics(   unsigned int _nx,
                    unsigned int _nu, 
                    unsigned int _ny,
                    VectorXf _initState,
                    float _samplingTime, 
                    float _initTime);
        
        /** Copy constructor
		 *
		 *	@param[in] rhs	Right-hand side object.
		 */
		dynamics( const dynamics& rhs );

		/** Destructor
		 */
		virtual ~dynamics( );


        /** Update system state given an input and return output
         * 
         * @param[in] _u        Control input
         * @param[in] _y        System output
         */
        void step( const VectorXf& _u, VectorXf& _y );


        /** Reset system to initial state
         */
        void resetDynamics( const VectorXf& offsets=VectorXf::Zero( 100 ) );


    //
	// PUBLIC DATA MEMBERS:
	//
        float time;             // Current time
        float samplingTime;     // Sampling time

        VectorXf state;         // System state
        float omega;            // Steppr motor rotational speed
        


    //
	// PRIVATE MEMBER FUNCTIONS:
	//
    private:
        /** Update system state using RK45
         * 
         * @param[in] _u        Control input
         */
        void updateState( const VectorXf& _u );

        
        /** Calculate state derivatives (rhs of equations of motion)
         * 
         * @param[in] _t        Current time
         * @param[in] _state    Current state
         * @param[in] _u        Control input
         * 
         * \return stateDerivative
         */
        VectorXf rocketDynamics( float _t, VectorXf _state, const VectorXf& _u );


    //
	// PRIVATE DATA MEMBER:
	//
        unsigned int nx;                // Number of states
        unsigned int nu;                // Number of inputs
        unsigned int ny;                // Number of outputs

        double p00 = 0.4165;			// Cd surface fit power coefficient
        double p10 = 8.886;				// Cd = p00       + p10 * x     + p01 * y     + p20 * x^2 + p11 * x*y
        double p01 = 0.3778;			//      p02 * y^2 + p21 * x^2*y + p12 * x*y^2 + p03 * y^3
        double p20 = 43.25;				// with x: airbrake extension, y: mach number
        double p11 = -10.48;
        double p02 = -0.9093;
        double p21 = 21.67;
        double p12 = 22.7;
        double p03 = 0.5587;

        float g = 9.81;                 // Gravitational constant
        float density_sea = 1.225;      // Sea-level density
        float A = 0.0191;               // Cross-sectional area
        float mass = 20.1;              // Mass at burn-out

        float V;                        // Velocity magnitude
        float M;                        // Mach number
        float Cd_val;                   // Drag coefficient
        float density;                  // Atmospheric density
        float initTime;                 // Initial time
        VectorXf initState;             // Initial state
        VectorXf lastU;                 // Previous control input

        // Runge-Kutta 45 integration
        VectorXf k1;
        VectorXf k2;
        VectorXf k3;
        VectorXf k4;
        VectorXf stateDerivative;    
};