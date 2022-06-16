#include "header.h"


int main(int argc, char const *argv[])
{
    /* Controller */
    PIDcontroller PID( 2, 1, 0.05 );

    // Controller gains
   	VectorXf pGains( 2 );
	pGains(0) = -3.0;     // -2.933
	pGains(1) = -3.0;       // -3.0

	VectorXf iGains( 2 );
	iGains(0) = 0.0;
	iGains(1) = 0.0;

	VectorXf dGains( 2 );
	dGains(0) = -7.0;     // -6.933
	dGains(1) = -7.0;       // -7.0

    PID.setProportionalGains( pGains );
    PID.setIntegralGains( iGains );
    PID.setDerivativeGains( dGains );

    // Controller limits
    PID.setControlLowerLimit(0, 0.0);
    PID.setControlUpperLimit(0, 0.05);

    PID.setControlLowerRateLimit(0, -0.05);
    PID.setControlUpperRateLimit(0, 0.05);

    // Controller reference using polynomial approximation coefficients
    MatrixXf ref( 2,6 );

    ref.row(0) << 0.000552959959483582,-0.0536167708516457,2.11969221432553,-48.1791794728476,707.049841776998,-1640.19371969719;   // Altitude reference signal
    ref.row(1) << -1.78444699910487e-05,0.00365544263025656,-0.225186229775288,6.27114893267685,-94.0507177583389,696.642796048007; // Velocity reference signal       

    PID.setPolynomialReference(ref);


    /* System dynamics */
    VectorXf init_state(4);
    init_state << 171.9, 1098.5, 54.14, 332.26;                         // Initial state
    float t_burn = 5.5;                                                 // Initial time

    unsigned int nx = 4;     // state dimension
    unsigned int nu = 1;     // Input dimension
    unsigned int ny = 2;     // Output dimension

    dynamics Rocket( nx, nu, ny, init_state, 0.05, t_burn );


    /* Closed-loop simulation */
    simulator Simulator( nx, nu, ny, PID, Rocket, 0.05 );

    //Simulator.simulate( 20.0, true );
    //Simulator.tune(  );
    Simulator.robustness( init_state );
}