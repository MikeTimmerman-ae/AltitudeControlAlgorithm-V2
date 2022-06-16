/**
 *	\file header.hpp
 *	\author Mike Timmerman
 *	\version 4.0
 *	\date 2022
 */

#pragma once

#include <iostream>                 // #include directives
#include <fstream>
#include <math.h>
#include <string>

#include "include/saturator.h"      // #include src code
#include "include/controller.h"     // #include src code
#include "include/controller.ipp"
#include "include/dynamics.h"       // #include src code
#include "include/simulator.h"      // #include src coude

#include "include/helpers.h"        // #include src coude

#include <Eigen/Dense>
#include <Eigen/SparseCore>

using namespace std;
using namespace Eigen;
