#include "header.h"

MatrixXf loadFile(string FileName, int row, int col) {
    MatrixXf Matrix(row, col);
	ifstream File(FileName);
	for (int k = 0; k < row; ++k) {
		string line;
		getline(File, line, '\n');
		for (int j = 0; j < col; ++j) {
			string entry = line.substr(0, line.find(','));
			line.erase(0, line.find(',') + 1);
			Matrix(k, j) = stod(entry);
		}
	}
    return Matrix;
}


int main(int argc, char const *argv[])
{
    /* System dynamics */
    dynamics Rocket;
    int n = 3;      // lifted dimension
    int nx = 2;     // state dimension
    int nrbf = 1;   // augmented dimension
    int nu = 1;     // Input dimension
    int ny = 2;     // Output dimension
    int N = 1;      // Prediction Horizon

    /* Lifting mapping function */
    MatrixXf cent = loadFile("../data/cent.csv", Rocket.n_states, nrbf);
    string rbf_type = "gauss";
    rbf liftFun(cent, rbf_type, 1.0/3510.0, 1.0);

    /* Linearized model */
    MatrixXf A = loadFile("../data/Alift.csv", n, n);               // x_dot = A*x + B*u
    MatrixXf B = loadFile("../data/Blift.csv", n, nu);                
    MatrixXf C = MatrixXf::Identity(ny, n);                         // y = C*x

    /* Define quadratic cost function */
    MatrixXf Q = MatrixXf::Identity(ny, n); Q(0,0) = 500; Q(1,1) = 10;
    MatrixXf R = MatrixXf::Identity(ny, n);
    MatrixXf P = MatrixXf::Identity(ny, n);

    /* Configure controller */
    

    /* Closed-loop simulation */
    int Tsim = 18;
    int Nsim = (int) Tsim/Rocket.dt;

    Rocket.state << 1074.63, 307.645;                                        // Initial state
    Rocket.t = Rocket.t_burn;                                               // Initial time
    VectorXf xlift;
    
    MatrixXf X(Nsim+1, Rocket.n_states); X(0, seq(0, Rocket.n_states - 1)) = Rocket.state.transpose();

    for (int i = 0; i < Nsim; ++i) {

        // Simulate closed loop feedback control
        VectorXf state = Rocket.state;
        xlift = liftFun.lift(state);
        Rocket.update_state(0);
                
        // Store data
        X(i+1, seq(0, Rocket.n_states-1)) = Rocket.state.transpose();

        if ((i+1)%25 == 0) {
            cout << "Altitude: " << Rocket.state[0] << " Time: " << Rocket.t << endl;
            cout << "Closed-Loop simulation: iteration " << i+1 << " out of " << Nsim << endl;
        }
    }

}