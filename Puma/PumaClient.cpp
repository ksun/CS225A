// PumaClient.cpp : Defines the entry point for the console application.
//

#ifdef WIN32
#pragma once
#endif

#include "PumaClient.hpp"

using namespace std;

bool posReached_joint(float *qd, float *q)
{
	float qTolerance = 0;
	static int reachedCntr = 0;
	for(int i = 0; i < DOF; i ++)
	{
		qTolerance += (qd[i]-q[i])*(qd[i]-q[i]);
	}
	if(qTolerance < QREACHEDTOL)
		reachedCntr ++;
	else
		reachedCntr = 0;

	if(reachedCntr >= QREACHEDITER)
	{
		reachedCntr = 0;
		return true;
	}
	return false;
}

bool posReached_OP(float *xd, float *x)
{
	float xTolerance = 0;
	static int reachedCntr = 0;
	for(int i = 0; i < DOF; i ++)
	{
		xTolerance += (xd[i]-x[i])*(xd[i]-x[i]);
	}
	if(xTolerance < XREACHEDTOL)
		reachedCntr ++;
	else
		reachedCntr = 0;

	if(reachedCntr >= QREACHEDITER)
	{
		reachedCntr = 0;
		return true;
	}
	return false;
}

// Move to joint position via jgoto
void MoveJGOTO(RobotCom *Robot, float *qd, float *q, float *dq)
{
	// Output the joint command
	Robot->jointControl(JGOTO, qd[0], qd[1], qd[2], qd[3], qd[4], qd[5]);

	// Wait for the robot to finish motion
	do
	{
		Robot->getStatus(GET_JPOS,q);
		Robot->getStatus(GET_JPOS,dq);
	}while(!posReached_joint(qd,q));
}

// Move to joint position via goto
void Move_GOTO(RobotCom *Robot, float *xd, float *x, float *dx)
{
	// Output the joint command
	Robot->control(GOTO, xd, 7);

	// Wait for the robot to finish motion
	do
	{
		Robot->getStatus(GET_IPOS,x);
		Robot->getStatus(GET_IPOS,dx);
	}while(!posReached_OP(xd,x));
}

void Move_GOTO_fast(RobotCom *Robot, float *xd)
{
	// Output the joint command
	Robot->control(GOTO, xd, 7);
}


