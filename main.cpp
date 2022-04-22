#include <iostream>                 // #include directives
#include <fstream>
#include <vector>
#include <math.h>
#include <string>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main(int argc, char const *argv[])
{

    MatrixXf A = MatrixXf::Constant(5, 2, 1.5);

    cout << A << endl;

}