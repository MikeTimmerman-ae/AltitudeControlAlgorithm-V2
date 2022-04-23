/**
 *	\file header.hpp
 *	\author Mike Timmerman
 *	\version 4.0
 *	\date 2022
 */

#pragma once

#include <iostream>                 // #include directives
#include <fstream>
#include <vector>
#include <math.h>
#include <string>

#include "include/dynamics.h"       // #include src coude
#include "include/rbf.h"       // #include src coude

#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include <Eigen/SparseCore>


using namespace std;
using namespace Eigen;