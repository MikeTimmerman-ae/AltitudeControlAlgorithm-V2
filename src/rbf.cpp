/**
 *	\file src/rbf.cpp
 *	\author Mike Timmerman
 *	\version 4.0
 *	\date 2022
 */

#include <string>              // #include directives        
#include <iostream>

#include "../include/rbf.h"         // #include header

#include <Eigen/Dense>              // #include modules

using namespace std;
using namespace Eigen;

rbf::rbf(MatrixXf &cent, string rbf_typeP, float epsP, float kP) {
    // Initialize the lifting function
    C = cent;
    Nrbf = cent.cols();
    rbf_type = rbf_typeP;
    eps = epsP; k = kP;
}

VectorXf rbf::lift(VectorXf &x) {
    // Create lifted matrix
    int Nstate = x.rows();
    VectorXf Y(Nrbf + Nstate, 1);

    // Populate lifted data matrix
    for (int i = Nstate; i < Nrbf + Nstate; ++i) {
        MatrixXf Cstate = C(seq(0, Nstate-1), i-Nstate);
        float r_squared = (x-Cstate).dot(x-Cstate);
        float y;

        if (rbf_type == "thinplate") {
            y = r_squared*log(sqrt(r_squared));
        } else if (rbf_type == "gauss") {
            y = exp(- powf(eps, 2.0) * r_squared);
        } else if (rbf_type == "invquad") {
            y = 1 / (1 + powf(eps, 2) * r_squared);
        } else if (rbf_type == "invmultquad") {
            y = 1 / sqrt(1 + powf(eps,2) * r_squared);
        } else if (rbf_type == "polyharmonic") {
            y = powf(r_squared, k/2) * log(sqrt(r_squared));
        } else {
            cout << "RBF type not recognized";
        } 
        if (y == NAN) {
            y = 0;
        }

        Y(i) = y;
    }
    Y(seq(0, Nstate-1)) = x;
    
    return Y;
}
