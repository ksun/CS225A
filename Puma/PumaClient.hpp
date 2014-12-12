// PumaClient.cpp : Defines the entry point for the console application.
//

#ifdef WIN32
#pragma once
#endif

#include "RobotCom.h"
#include <iostream>
#include <math.h>

using namespace std;
#define DOF 6
#define QREACHEDTOL 10 // fine tune this parameter
#define XREACHEDTOL 10 //fine tune this parameter
#define QREACHEDITER 100 // fine tune this parameter

bool posReached_joint(float *qd, float *q);

bool posReached_OP(float *xd, float *x);

// Move to joint position via jgoto
void MoveJGOTO(RobotCom *Robot, float *qd, float *q, float *dq);

// Move to joint position via goto
void Move_GOTO(RobotCom *Robot, float *xd, float *x, float *dx);


void Move_GOTO_fast(RobotCom *Robot, float *xd);


