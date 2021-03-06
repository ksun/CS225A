// PumaClient.cpp : Defines the entry point for the console application.
//

#ifdef WIN32
#pragma once
#endif

#include "PumaClient.hpp"

/*
//scl lib
#include <scl/DataTypes.hpp>
#include <scl/data_structs/SGcModel.hpp>
#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl/dynamics/tao/CDynamicsTao.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/graphics/chai/CGraphicsChai.hpp>
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>
#include <scl/util/DatabaseUtils.hpp>

//#include "chai3d.h"

//For timing
#include <sutil/CSystemClock.hpp>

//Eigen 3rd party lib
#include <Eigen/Dense>
 */
//Standard includes (for printing and multi-threading)
#include <iostream>
#include <omp.h>

//Free glut windowing environment
#include <GL/freeglut.h>
#include <math.h>

// Computer Vision
#include "../Kinect/ball_tracking.hpp"

using namespace std;
#define DOF 6
#define QREACHEDTOL 10 // fine tune this parameter
#define XREACHEDTOL 10 //fine tune this parameter
#define QREACHEDITER 100 // fine tune this parameter

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

int main(int argc, const char* argv[])
{

	// declarations
	RobotCom* PumaRobot = new RobotCom();
	float q_desired_[DOF] = {0,0,0,0,0,0}; // in degrees
	float dq_[DOF], q_[DOF];
	char key = 'a';
	// float the robot right after connection is established
	PumaRobot->_float();
	cout << "Float..." << endl;
	cout << "Disengage E-stop and hit 's'" << endl;
	while(1)
	{
		cin >> key;
		if(key == 's')
			break;
	}
	cout << "Disengaged..." << endl;
	cout << "Starting tasks..." << endl;
	// start tasks
	while(1)
	{
		/********************************************************/
		// first motion
		/********************************************************/
		q_desired_[2] = 30;
		q_desired_[3] = 10;
		q_desired_[4] = 15;
		MoveJGOTO(PumaRobot,q_desired_,q_,dq_);
		cout << "Motion 1 done" << endl;
		break;
	}

	/********************************************************/
	// Start Doing our project
	/********************************************************/
	cout << "Want to fine tune the joint positions? y/n "<< endl;
	cin >> key;
	if(mode == 'y'){
		while(1){
			endl;

			PumaRobot->getStatus(GET_JPOS,q_);
			int num = 5;
			cout << "Which joint you want to change? 0-5" << endl;
			cin >> num;
			if(num < 6 && num >=0){
				cout << "q[" << num << "] (present value : " << q_[num] << ") :" << endl;
				cin >> q_desired_[num];
				MoveJGOTO(PumaRobot,q_desired_,q_,dq_);
				PumaRobot->getStatus(GET_JPOS,q_fine_tune);
				PumaRobot->getStatus(GET_IPOS,x_fine_tune);
			}
		}
	}

	cout << "Go to the fine tune position? y/n" << endl;

	cin >> key;
	if(key=='y') MoveJGOTO(PumaRobot,q_fine_tune,q_,dq_);


	cout << "Start Doing our project. Position Ball. Press any key to continue." << endl;
	cin >> key;
	if(key == 'q')
		break;


	while(1)
	{

		if ( !valid || markers.size() <= 0 ) break;

		//*
		X_ball = markers[0] + offset;
		V_ball = position-last_position;




		ti = (V_ball(2) + sqrt(V_ball(2)*V_ball(2) + 2*9.8*(X_ball(2)-zi)))/9.8;


		if(0.1 > abs(ti)){


			if((X_ball(2) < (zi+0.2)) && (V_ball(2) < 0)){
				x_des << X_ball(0)+V_ball(0)*dt, X_ball(1)+V_ball(1)*dt, zi+0.05;

			}

			else if((X_ball(2) > zi) && (V_ball(2) >= 0)){
				x_des << X_ball(0)+V_ball(0)*dt, X_ball(1)+V_ball(1)*dt, zi-0.01;

			}
		} else if(0 < ti){

			//Move the end effector to be under the predict (x,y) position of ball.
			x_des << X_ball(0)+V_ball(0)*ti, X_ball(1)+V_ball(1)*ti, zi-0.01;




		}
		Robot->getStatus(GET_IPOS, x_curr);
		if (x_des(0) >= 0 && x_des(1) >= 0 && (x_des - x_curr).norm() < 0.7){
			Move_GOTO_fast(Robot, x_des);
		}
	}

	// float the before disconnecting
	PumaRobot->_float();

	//Disconnect Robot

	//Sleep(2000);
	//PumaRobot->~RobotCom();
}
}
return 0;
}
