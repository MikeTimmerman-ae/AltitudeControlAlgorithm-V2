/**
 *	\file include/controller.ipp
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#include "../header.h"    // #include header


inline void PIDcontroller::getU(   VectorXf& _u    )
{
    _u = u;
}