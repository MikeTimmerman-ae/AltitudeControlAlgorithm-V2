/**
 *	\file src/dynamics.cpp
 *	\author Mike Timmerman
 *	\version 4.1
 *	\date 2022
 */

#include "../header.h"    // #include header


//
// PUBLIC MEMBER FUNCTIONS:
//

dynamics::dynamics(  ) {}


dynamics::dynamics( unsigned int _nx,
                    unsigned int _nu, 
                    unsigned int _ny,
                    VectorXf _initState,
                    float _samplingTime, 
                    float _initTime )
{
    // Initialize system dynamics properties
    nx = _nx;
    nu = _nu;
    ny = _ny;

    initTime = _initTime;
    time = _initTime;
    samplingTime = _samplingTime;

    bias = VectorXf::Zero( _ny );
    noiseLevel = VectorXf::Zero( _ny );

    lastU = VectorXf::Zero( _nu );
    initState = _initState;
    state = _initState;

    // Initialize private data members
    k1 = VectorXf::Zero(nx,1);
    k2 = VectorXf::Zero(nx,1);
    k3 = VectorXf::Zero(nx,1);
    k4 = VectorXf::Zero(nx,1);
    stateDerivative = VectorXf::Zero(nx,1);
}


dynamics::~dynamics(  ) {}


void dynamics::step( const VectorXf& _u, VectorXf& _y )
{
    /*  Update system state */
    updateState( _u );

    /* Update system output */
    VectorXf tmp( ny ); tmp << state[1], state[3];

    for (unsigned int i=0; i<ny; i++)
    {
        // Add noise
        tmp(i) = tmp(i)*(1+ noiseLevel(i)*( rand() % 201 - 100.0) / 100.0);

        // Add bais
        tmp(i) = tmp(i) + bias(i);
    }

    _y = tmp;
}


void dynamics::setBias( const VectorXf& _bias )
{
    bias = _bias;
}


void dynamics::setNoise( const VectorXf& _noiseLevel )
{
    noiseLevel = _noiseLevel;
}


void dynamics::resetDynamics( const VectorXf& offsets )
{   
    for (unsigned int i=0; i<nx; i++)
    {   
        state(i) = initState(i)*(1+offsets(i));
    }
    time = initTime;
    lastU = VectorXf::Zero( nu );
}



//
// PRIVATE MEMBER FUNCTIONS:
//

void dynamics::updateState( const VectorXf& _u )
{   
    omega = (_u(0)-lastU(0))/(samplingTime*0.01);
    lastU = _u;

    // Evaluation at start of interval
    k1 = rocketDynamics(time, state, _u);

    // Evaluation at midway of interval
    k2 = rocketDynamics(time + 1/2*samplingTime, state + samplingTime*k1/2.0, _u);

    k3 = rocketDynamics(time + 1/2*samplingTime, state + samplingTime*k2/2.0, _u);

    // Evaluation at end of interval
    k4 = rocketDynamics(time + samplingTime, state + samplingTime*k3, _u);

    // Update system dynamics properties   
    state = state + samplingTime*(k1 + 2.0*k2 + 2.0*k3 + k4)/6.0;
    time = time + samplingTime;
}


VectorXf dynamics::rocketDynamics( float _t, VectorXf _state, const VectorXf& _u )
{
    float xbr = _u(0);

    density = density_sea * exp(-_state[1] / 8000.0);
    V = sqrt(pow(_state[2], 2) + pow(_state[3], 2));
    M = V/sqrt(1.4*287*278);
    Cd_val = p00 + p10*xbr + p01*M + p20*xbr*xbr + p11*M*xbr + p02*M*M + p21*xbr*xbr*M + p12*xbr*M*M + p03*M*M*M;

    stateDerivative[0] = _state[2];               // x_dot = Vx
    stateDerivative[1] = _state[3];               // y_dot = Vy
    stateDerivative[2] = - 1.0/2.0 * density * A * Cd_val * V * _state[2] / mass;  // Vx_dot
    stateDerivative[3] = -g - 1.0/2.0 * density * A * Cd_val * V * _state[3] / mass;  // Vy_dot

    return stateDerivative;
}
