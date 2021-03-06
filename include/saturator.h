/**
 *	\file include/saturator.h
 *	\author Mike Timmerman
 *	\version 4.0
 *	\date 2022
 */

#pragma once

#include <Eigen/Dense>              // #include module
using namespace Eigen;              // using namespace of module

class saturator
{
    //
    // PUBLIC MEMBER FUNCTIONS
    //
    public:

        /** Default constructor
         */ 
        saturator();

        /** Constructor which takes dimensions of signal to be saturated
         *
         * @param[in] _nU                   Number of control signals to be saturated
         * @param[in] _samplingTime         Sampling time
         * @param[in] _bias                 Bias on control input
         * @param[in] _noiseLevel           Noise level on control input
         */
        saturator( unsigned int _nU, float _samplingTime );
        
        /** Copy constructor
		 *
		 *	@param[in] rhs	Right-hand side object.
		 */
		saturator( const saturator& rhs );

		/** Destructor. 
		 */
		~saturator( );


        /** Assigns new lower limit on control signals
         * 
         * @param[in] _lowerLimit       New lower limit on control signal
         */
        void setControlLowerLimit( const VectorXf& _lowerLimit );

        /** Assigns new lower limit on given component of control signal
         * 
         * @param[in] idx               Index of control signal component
         * @param[in] _lowerLimit       New lower limit
         */
        void setControlLowerLimit( unsigned int idx, float _lowerLimit );

        /** Assigns new upper limit on control signals
         * 
         * @param[in] _upperLimit       New upper limit on control signal
         */
        void setControlUpperLimit( const VectorXf& _upperLimit );

        /** Assigns new upper limit on given component of control signal
         * 
         * @param[in] idx               Index of control signal component
         * @param[in] _upperLimit       New upper limit
         */
        void setControlUpperLimit( unsigned int idx, float _upperLimit );


        /** Assigns new lower rate limit on control signals
         * 
         * @param[in] _lowerRateLimit   New lower rate limit on control signal
         */
        void setControlLowerRateLimit( const VectorXf& _lowerRateLimit );

        /** Assigns new lower rate limit on given component of control signal
         * 
         * @param[in] idx               Index of control signal component
         * @param[in] _lowerRateLimit   New lower rate limit
         */
        void setControlLowerRateLimit( unsigned int idx, float _lowerRateLimit );

        /** Assigns new upper rate limit on control signals
         * 
         * @param[in] _upperRateLimit   New upper limit on control signal
         */
        void setControlUpperRateLimit( const VectorXf& _upperRateLimit );

        /** Assigns new upper rate limit on given component of control signal
         * 
         * @param[in] idx               Index of control signal component
         * @param[in] _upperRateLimit   New upper limit
         */
        void setControlUpperRateLimit( unsigned int idx, float _upperRateLimit );


        /** Set bias on control signals
         * 
         * @param[in] _bias             Bias on control input
         */
        void setBias( const VectorXf& _bias );

        /** Set noise on control signals
         * 
         * @param[in] _noiseLevel               Index of control signal component
         */
        void setNoise( const VectorXf& _noiseLevel );


        /** Reset satuator
         */
        void resetSaturator(  );



    //
    // PROTECTED MEMBER FUNCTIONS
    //
    protected:

        /** Saturate the control signal
         * 
         * @param[in] _u                Control signal
         */
        void saturate( VectorXf& _u );



    //
    // PROTECTED DATA MEMBERS
    //

        unsigned int nU;                            // Number of control variables
        float samplingTime;                         // Controller time step

        VectorXf bias;                              // bias in control input 
        VectorXf noiseLevel;                        // percentage noise deviations

        VectorXf lastU;				                // Previous control

		VectorXf lowerLimitControls;				// Lower limits on control signals
		VectorXf upperLimitControls;				// Upper limits on control signals

		VectorXf lowerRateLimitControls;			// Lower rate limits on control signals
		VectorXf upperRateLimitControls;			// Upper rate limits on control signals

};
