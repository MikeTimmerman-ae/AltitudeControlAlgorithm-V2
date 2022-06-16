/**
 *	\file include/simulator.h
 *	\author Mike Timmerman
 *	\version 2.0
 *	\date 2022
 */

#pragma once

#include <Eigen/Dense>              // #include module
using namespace Eigen;              // using namespace of module

class simulator
{
    //
	// PUBLIC MEMBER FUNCTIONS:
	//
    public:

        /** Default onstructor
         */
        simulator();
        
        /** Set simulator parameters
         * 
         * @param[in] _nx           Number of states
         * @param[in] _nu           Number of inputs
         * @param[in] _ny           Number of outputs
         * @param[in] controller    PID Controller
         * @param[in] system        Dynamical system
         * @param[in] _samplingTime Sampling Time
         * 
         */
        simulator(  unsigned int _nx,
                    unsigned int _nu,
                    unsigned int _ny,
                    PIDcontroller& controller,
                    dynamics& system,
                    float _samplingTime );


        /** Simulate system given the controller and initial state
         * 
         * @param[in] simulationTime    // Simulation time
         * @param[in] saveData          // Indicate if data should be saved
         * 
         */
        void simulate( float simulationTime, bool saveData );

        /** Tune controller gains
         */
        void tune(  );

        /** Generate robustness map
         */
        void robustness( const VectorXf& initState );
        

    //
	// PRIVATE DATA MEMBER:
	//
    private:
        unsigned int nx;        // Number of states
        unsigned int nu;        // Number of inputs
        unsigned int ny;        // Number of outputs

        dynamics Rocket;        // Rocket dynamics
        PIDcontroller PID;      // PID controller
        
        MatrixXf X;             // Save state data
        MatrixXf U;             // Save input data

        float samplingTime;     // Sampling time

};
