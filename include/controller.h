/**
 *	\file include/controller.h
 *	\author Mike Timmerman
 *	\version 4.0
 *	\date 2022
 */

#pragma once

#include <Eigen/Dense>              // #include module
using namespace Eigen;              // using namespace of module

class PIDcontroller : public saturator
{
	//
	// PUBLIC MEMBER FUNCTIONS:
	//
    public:
		/** Default constructor. 
		 */
        PIDcontroller(  );

        /** Constructor which takes the number of inputs and outputs of the
		 *	PID controller as well as the sampling time.
         *  
         * @param[in] _nInputs          Number of inputs
         * @param[in] _nOuputs          Number of inputs
         * @param[in] _samplingTime     Sampling time
		 */
        PIDcontroller( unsigned int _nInputs, unsigned int _nOuputs, double _sampleTime );
        
        /** Copy constructor
		 *
		 *	@param[in] rhs	Right-hand side object.
		 */
		PIDcontroller( const PIDcontroller& rhs );

		/** Destructor
		 */
		virtual ~PIDcontroller( );


        /** Assign proportional gains to input components
         * 
         * @param[in] _pGains     New proportional weights
         */
        void setProportionalGains( const VectorXf& _pGains );

        /** Assign integral gains to input components
         * 
         * @param[in] _pGains     New integral weights
         */
        void setIntegralGains( const VectorXf& _iGains );
        
        /** Assign derivative gains to input components
         * 
         * @param[in] _pGains     New integral weights
         */
        void setDerivativeGains( const VectorXf& _dGains );


        /** Set reference trajectory using polynomial coefficients
         * 
         * @param[in] _refCoeff     Matrix containing nth order polynomial coefficients, one row per input signal
         *                          p = p_1*x^n + p_2*x^(n-1) + ... + p_n*x + p_{n+1}
         */
        void setPolynomialReference( const MatrixXf& _refCoeff );


        /** Initilizes the control law with given start values and performs consitency checks
         * 
         * @param[in] _startTime        Start time
         * @param[in] _x                Initial value for differential states
         * @param[in] _yRef             Initial value for reference trajectory
         */
        void init( const VectorXf& _x0, const VectorXf& _yRef, double startTime );

        /** Initilizes the control law with given start values and performs consitency checks
         * 
         * @param[in] _startTime        Start time
         * @param[in] _x                Initial value for differential states
         */
        void init( const VectorXf& _x0, double startTime );

        /** Perform step of control law based on inputs
         * @param[in] currentTime   Current time
         * @param[in] _x            Current value of differential states
         * @param[in] _yRef         Current reference trajectory
         * 
         */
        void step( double currentTime, const VectorXf& _x, const VectorXf& _yRef );

        /** Perform step of control law based on inputs and predefined reference
         * @param[in] currentTime   Current time
         * @param[in] _x            Current value of differential states
         * 
         */
        void step( double currentTime, const VectorXf& _x );


        /** Returns control signal as determined by control law
         * 
         * @param[out] _u   Control signal
         * 
         */
        inline void getU( VectorXf& _u );


        /** Reset controller to inital state
         */
        void resetController(  );

    //
	// PRIVATE DATA MEMBER:
	//
    private:
        unsigned int nInputs;           // Number of inputs
        unsigned int nOutputs;          // Number of Outputs

        VectorXf pGains;                // Proportional gains for all input components
        VectorXf iGains;                // Integral gains for all input components
        VectorXf dGains;                // Derivative gains for all input components

        VectorXf iValue;                // Integrated value for all input components
        VectorXf lastError;             // Last error input

        float samplingTime;

        MatrixXf refCoeff;              // Reference using polynomial coefficients
        VectorXf u;                     // Control input
    

    //
	// PRIVATE MEMBER FUNCTION:
	//
    private:
        /** Calculate current control action based on current error
         * 
         * @param[in] error     Current error
         * @param[out] output   Current control action
         */
        void determineControlAction( const VectorXf& error, VectorXf& output );
};
