/**
 *	\file src/simulator.cpp
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#include "../header.h"    // #include header


//
// PUBLIC MEMBER FUNCTIONS:
//

simulator::simulator(  ){}


simulator::simulator(   unsigned int _nx,
                        unsigned int _nu,
                        unsigned int _ny,
                        PIDcontroller& controller,
                        dynamics& system,
                        float _samplingTime )
{
    nx = _nx;
    nu = _nu;
    ny = _ny;

    Rocket = system;
    PID = controller;
    
    samplingTime = _samplingTime;
}


void simulator::simulate( float simulationTime, bool saveData )
{   
    // Simulation points
    int Nsim = (int) simulationTime/Rocket.samplingTime;

    // Determine data saving
    if (saveData)
    {        
        X = MatrixXf::Zero(nx+1, Nsim+1); X(seq(0, nx-1), 0) = Rocket.state;
        Y = MatrixXf::Zero(ny, Nsim+1); X(0, 0) = Rocket.state[1]; X(1, 0) = Rocket.state[3];
        U = MatrixXf::Zero(1, Nsim+1); U(0, 0) = 0.0;
    }
    else
        X = MatrixXf::Zero(1, Nsim+1); X(0, 0) = Rocket.state(1);

    // Control and output vectors
    VectorXf u;
    VectorXf y(ny); y << Rocket.state[1], Rocket.state[3];
    
    // Initialize controller
    PID.init( y, Rocket.time );

    // Run closed-loop simulation
    for (int i = 0; i < Nsim; ++i)
    {
        PID.step( Rocket.time, y );
        PID.getU( u );
        Rocket.step( u,y );

        if (saveData)
        {
            // Store data
            X(seq(0, nx-1), i+1) = Rocket.state;
            X(nx, i+1) = Rocket.omega;
            Y(seq(0, ny-1), i+1) = y;
            U(0, i+1) = u(0);

            if ((i+1)%25 == 0)
            {
                std::cout << "Altitude: " << y(0) << " Time: " << Rocket.time << std::endl;
                std::cout << "Closed-Loop simulation: iteration " << i+1 << " out of " << Nsim << std::endl;
            }
        } 
        else
            X(0, i+1) = y(0);       // Store altitude data
    }

    // Export data
    if (saveData)
    {
        saveToFile(X, X.rows(), X.cols(), "../data/state.csv");
        saveToFile(Y, Y.rows(), Y.cols(), "../data/output.csv");
        saveToFile(U, U.rows(), U.cols(), "../data/input.csv");
    }
}


void simulator::tune(  )
{   
    unsigned int k = 1;                 // counter
    float res = 0.1;                    // gain resolution
    float bestDev = 1000.0;             // best obtained target deviation
    VectorXf gains(3);                  // corresponding gains^

    for (int i=-20; i <= 20.0; i++) {
        for (int ii=0; ii <= 0; ii++) {
            for (int iii=-20; iii <=20.0; iii++) {
                /* Reset controller and dynamics */
                Rocket.resetDynamics();
                PID.resetController();
                PID.resetSaturator();

                /* Vary controller gains */
                VectorXf pWeights( 2 );
                pWeights(0) = -3.0;
                pWeights(1) = i*res;

                VectorXf iWeights( 2 );
                iWeights(0) = 0.0;
                iWeights(1) = ii*res;

                VectorXf dWeights( 2 );
                dWeights(0) = -7.0;
                dWeights(1) = iii*res;

                PID.setProportionalGains( pWeights );
                PID.setIntegralGains( iWeights );
                PID.setDerivativeGains( dWeights );

                /* Closed-loop simulation */
                simulate( 20.0,false );

                std::cout << "Round: " << k << std::endl;
                std::cout << "Difference: " << abs( 3500 - X.maxCoeff() ) << std::endl;
                k++;

                if ( abs( 3500 - X.maxCoeff()) < bestDev )
                {
                     bestDev = abs( 3500 - X.maxCoeff() );
                     gains[0] = i*res; gains[1] = ii*res; gains[2] = iii*res;
                }

            }
        }
    }   
    std::cout << "Smallest deviation: " << bestDev << std::endl;
    std::cout << "For combintation of gains: " << gains << std::endl;
}


void simulator::robustness( const VectorXf& initState )
{
    nx = initState.size();
    unsigned int k = 0;
    MatrixXf deviations(441,1);
    MatrixXf stateOffsets(441,4);

    for (int i=-10; i <= 10; i++) {
        for (int ii=-10; ii <= 10; ii++) {
            /* Reset controller and dynamics */
            VectorXf offsets(4);                        // State percentage offsets
            offsets(0) = 0.0;
            offsets(1) = i*0.10/10.0;
            offsets(2) = 0.0;
            offsets(3) = ii*0.10/10.0;

            Rocket.resetDynamics( offsets );
            PID.resetController();
            PID.resetSaturator();

            /* Closed-loop simulation */
            simulate( 20.0,false );

            /* Save data */
            deviations(k,0) = 3500 - X.maxCoeff();
            for (unsigned i=0; i<nx; i++)
                stateOffsets(k,i) = initState(i)*offsets(i);
 
            std::cout << "Round: " << k+1 << " offsets " << i << ii << std::endl;
            std::cout << "Difference: " << abs( 3500 - X.maxCoeff() ) << std::endl;            
            k++;
        }
    }   
    saveToFile(deviations, deviations.rows(), deviations.cols(), "../data/deviations.csv");
	saveToFile(stateOffsets, stateOffsets.rows(), stateOffsets.cols(), "../data/stateOffsets.csv");
}