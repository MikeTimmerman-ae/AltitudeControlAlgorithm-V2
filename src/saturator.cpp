/**
 *	\file src/saturator.cpp
 *	\author Mike Timmerman
 *	\version 4.0
 *	\date 2022
 */

#include "../header.h"    // #include header


//
// PUBLIC MEMBER FUNCTIONS:
//

saturator::saturator(  ){}


saturator::saturator( unsigned int _nU, float _samplingTime )
{   
    lowerLimitControls = VectorXf::Ones( _nU )*1000000;
    upperLimitControls = VectorXf::Ones( _nU )*1000000;

    lowerRateLimitControls = VectorXf::Ones( _nU )*1000000;
    upperRateLimitControls = VectorXf::Ones( _nU )*1000000;

    bias = VectorXf::Zero( _nU );
    noiseLevel = VectorXf::Zero( _nU );

    nU = _nU;
    samplingTime = _samplingTime;
    lastU = VectorXf::Zero( _nU );
}


saturator::saturator( const saturator& rhs )
{
    lowerLimitControls = rhs.lowerLimitControls;
    upperLimitControls = rhs.upperLimitControls;

    lowerRateLimitControls = rhs.lowerRateLimitControls;
    upperRateLimitControls = rhs.upperRateLimitControls;

    bias = rhs.bias;
    noiseLevel = rhs.noiseLevel;

    nU = rhs.nU;
    samplingTime = rhs.samplingTime;
    lastU = rhs.lastU;
}


saturator::~saturator(  ){}



void saturator::setControlLowerLimit( const VectorXf& _lowerLimit )
{
    if ( _lowerLimit.size() != nU )
        throw std::invalid_argument("Incorrect number of control limits given");
    
    lowerLimitControls = _lowerLimit;
}

void saturator::setControlLowerLimit( unsigned int idx, float _lowerLimit )
{
    if ( idx >= nU )
        throw std::invalid_argument("Invalid index for control signal given");
    
    lowerLimitControls( idx ) = _lowerLimit;
}

void saturator::setControlUpperLimit( const VectorXf& _upperLimit )
{
    if ( _upperLimit.size() != nU )
        throw std::invalid_argument("Incorrect number of control limits given");
    
    upperLimitControls = _upperLimit;
}

void saturator::setControlUpperLimit( unsigned int idx, float _upperLimit )
{
    if ( idx >= nU )
        throw std::invalid_argument("Invalid index for control signal given");
    
    upperLimitControls( idx ) = _upperLimit;
}


void saturator::setControlLowerRateLimit( const VectorXf& _lowerRateLimit )
{
    if ( _lowerRateLimit.size() != nU )
        throw std::invalid_argument("Incorrect number of control rate limits given");
    
    lowerRateLimitControls = _lowerRateLimit;
}

void saturator::setControlLowerRateLimit( unsigned int idx, float _lowerRateLimit )
{
    if ( idx >= nU )
        throw std::invalid_argument("Invalid index for control signal given");
    
    lowerRateLimitControls( idx ) = _lowerRateLimit;
}

void saturator::setControlUpperRateLimit( const VectorXf& _upperRateLimit )
{
    if ( _upperRateLimit.size() != nU )
        throw std::invalid_argument("Incorrect number of control rate limits given");
    
    upperRateLimitControls = _upperRateLimit;
}

void saturator::setControlUpperRateLimit( unsigned int idx, float _upperRateLimit )
{
    if ( idx >= nU )
        throw std::invalid_argument("Invalid index for control signal given");
    
    upperRateLimitControls( idx ) = _upperRateLimit;
}


void saturator::setBias( const VectorXf& _bias )
{
    bias = _bias;
}

void saturator::setNoise( const VectorXf& _noiseLevel )
{
    noiseLevel = _noiseLevel;
}


void saturator::resetSaturator(  )
{
    lastU = VectorXf::Zero( nU );
}



//
// PROTECTED MEMBER FUNCTIONS:
//

void saturator::saturate( VectorXf& _u )
{   
    // consistency check
    if ( _u.size() != nU )
        throw std::invalid_argument("Incorrect number of control signals given");

    // Set upper and lower bounds
    VectorXf Uub( upperLimitControls );
    VectorXf Ulb( lowerLimitControls );

    for ( unsigned int i=0; i<nU; ++i )
    {
        if (lastU(i) + samplingTime*lowerRateLimitControls(i) > Ulb(i))
            Ulb(i) = lastU(i) + samplingTime*lowerRateLimitControls(i);
        if (lastU(i) + samplingTime*upperRateLimitControls(i) < Uub(i))
            Uub(i) = lastU(i) + samplingTime*upperRateLimitControls(i);
    }

    // Update control input
    for ( unsigned int i=0; i<nU; ++i )
    {
        if (_u(i) > Uub(i))
            _u(i) = Uub(i);
        if (_u(i) < Ulb(i))
            _u(i) = Ulb(i);
    }
    lastU = _u;

    for ( unsigned int i=0; i<nU; i++ )
    { 
        // Add bias 
        _u(i) = _u(i) + bias(i);
        
        if (_u(i) < lowerLimitControls(i))
            _u(i) = lowerLimitControls(i);
        if (_u(i) > upperLimitControls(i))
            _u(i) = upperLimitControls(i);
        
        // Add noise
        _u(i) = _u(i)*(1+ noiseLevel(i)*( rand() % 201 - 100.0) / 100.0);
    }
}