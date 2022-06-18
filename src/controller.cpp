/**
 *	\file src/controller.cpp
 *	\author Mike Timmerman
 *	\version 4.0
 *	\date 2022
 */

#include "../header.h"    // #include header


//
// PUBLIC MEMBER FUNCTIONS:
//

PIDcontroller::PIDcontroller(  ) : saturator(  )
{
    nInputs = 0;
    nOutputs = 0;
}


PIDcontroller::PIDcontroller(   unsigned int _nInputs,
                                unsigned int _nOutputs,
                                float _sampleTime ) : saturator( _nOutputs, _sampleTime )
{
    if ( ( _nOutputs != _nInputs ) && ( _nOutputs != 1 ) )
    {
        _nOutputs = 1;
    }

    nInputs  = _nInputs;
	nOutputs = _nOutputs;

	pGains = VectorXf::Zero( nInputs );
	iGains = VectorXf::Zero( nInputs );
	dGains = VectorXf::Zero( nInputs );

    iValue = VectorXf::Zero( nInputs );
	lastError = VectorXf::Zero( nInputs );

    samplingTime = _sampleTime;
}


PIDcontroller::PIDcontroller( const PIDcontroller& rhs ) : saturator( rhs )
{
    nInputs  = rhs.nInputs;
	nOutputs = rhs.nOutputs;

	pGains = rhs.pGains;
	iGains = rhs.iGains;
	dGains = rhs.dGains;

	iValue    = rhs.iValue;
	lastError = rhs.lastError;
}


PIDcontroller::~PIDcontroller(  ){}


void PIDcontroller::setProportionalGains( const VectorXf& _pGains )
{
    if ( _pGains.size() != nInputs )
        throw std::invalid_argument("Number of proportional gains does not match number of inputs");
    else
        pGains = _pGains;
}


void PIDcontroller::setIntegralGains( const VectorXf& _iGains )
{
    if ( _iGains.size() != nInputs )
        throw std::invalid_argument("Number of integral gains does not match number of inputs");
    else
        iGains = _iGains;
}


void PIDcontroller::setDerivativeGains( const VectorXf& _dGains )
{
    if ( _dGains.size() != nInputs )
        throw std::invalid_argument("Number of derivative gains does not match number of inputs");
    else
        dGains = _dGains;
}


void PIDcontroller::setPolynomialReference( const MatrixXf& _refCoeff)
{
    if ( _refCoeff.rows() != nInputs )
        throw std::invalid_argument("Incorrect number of reference trajectories given");
    else
        refCoeff = _refCoeff;
}


void PIDcontroller::init( const VectorXf& _x0, const VectorXf& _yRef, double startTime )
{
    if ( _x0.size() != nInputs ) 
        throw std::invalid_argument("Incorrect number of input dimensions");

    // Set reference trajectory
    VectorXf xRef( _x0.size() );

    if ( _yRef.size() > 0 )
    {
        if ( _yRef.size() != nInputs )
            throw std::invalid_argument("Incorrect number of reference trajectories given");
        else
            xRef = _yRef;
    }
    else
    {
        xRef.setZero();
    }

    // Initialize control signals
    u = VectorXf::Zero( nOutputs );

    lastError = xRef - _x0;
}


void PIDcontroller::init( const VectorXf& _x0, double startTime )
{
    if ( _x0.size() != nInputs ) 
        throw std::invalid_argument("Incorrect number of input dimensions");

    // Get reference trajectory
    VectorXf xRef( _x0.size() ); xRef.setZero();

    if ( refCoeff.rows() > 0 )
    {
        if ( refCoeff.rows() != nInputs )
            throw std::invalid_argument("Incorrect number of reference trajectories given");
        else
            for ( unsigned int i=0; i<refCoeff.rows(); ++i)
            {
                for ( unsigned int j=0; j<refCoeff.cols(); ++j)
                {
                    xRef(i) += refCoeff(i,j) * pow( startTime,refCoeff.cols()-j-1 );
                }
            }
    }
    else
    {
        xRef.setZero();
    }

    // Initialize control signals
    u = VectorXf::Zero( nOutputs );

    lastError = xRef - _x0;
}


void PIDcontroller::step( double currentTime, const VectorXf& _x, const VectorXf& _yRef )
{
    if ( _x.size() != nInputs ) 
        throw std::invalid_argument("Incorrect number of input dimensions");

    // Set reference trajectory
    VectorXf xRef( _x.size() );

    if ( _yRef.size() > 0 )
    {
        if ( _yRef.size() != nInputs )
            throw std::invalid_argument("Incorrect number of reference trajectories given");
        else
            xRef = _yRef;
    }
    else
    {
        xRef.setZero();
    }

    // Determine PID control action
    if ( nOutputs > 0 )
    {
        determineControlAction( xRef - _x,u );
    }
    else
        u.setZero();

    // Saturate output
    saturate( u );
}


void PIDcontroller::step( double currentTime, const VectorXf& _x )
{
    if ( _x.size() != nInputs ) 
        throw std::invalid_argument("Incorrect number of input dimensions");

    // Get reference trajectory
    VectorXf xRef( _x.size() ); xRef.setZero();

    if ( refCoeff.rows() > 0 )
    {
        if ( refCoeff.rows() != nInputs )
            throw std::invalid_argument("Incorrect number of reference trajectories given");
        else
            for ( unsigned int i=0; i<refCoeff.rows(); ++i)
            {
                for ( unsigned int j=0; j<refCoeff.cols(); ++j)
                {
                    xRef(i) += refCoeff(i,j) * pow( currentTime,refCoeff.cols()-j-1 );
                }
            }
    }
    else
    {
        xRef.setZero();
    }

    // Determine PID control action
    if ( nOutputs > 0 )
    {
        determineControlAction( xRef - _x,u );
    }
    else
        u.setZero();

    // Saturate output
    saturate( u );
}


void PIDcontroller::resetController(  )
{
    iValue = VectorXf::Zero( nInputs );
	lastError = VectorXf::Zero( nInputs );
    
    u = VectorXf::Zero( nOutputs );
}



//
// PRIVATE MEMBER FUNCTION:
//

void PIDcontroller::determineControlAction( const VectorXf& error, VectorXf& output )
{
    unsigned int i;
    double tmp;

    output = VectorXf::Zero( nOutputs );

    // Update integral value
    for ( i=0; i<nInputs; ++i )
        iValue(i) += error(i) * samplingTime;

    // determine ouputs
    for ( i=0; i<nInputs; ++i )
    {
        tmp  = pGains(i) * error(i);
        tmp += iGains(i) * iValue(i);
        tmp += dGains(i) * (error(i) - lastError(i)) / samplingTime;

        if ( nOutputs > 1  )
            output(i) = tmp;
        else
            output(0) += tmp;
    }

    // update last error
    lastError = error;
}
