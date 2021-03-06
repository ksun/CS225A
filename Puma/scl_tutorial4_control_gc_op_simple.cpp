/* This file is part of scl, a control and simulation library
   for robots and biomechanical models.

   scl is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 3 of the License, or (at your option) any later version.

   Alternatively, you can redistribute it and/or
   modify it under the terms of the GNU General Public License as
   published by the Free Software Foundation; either version 2 of
   the License, or (at your option) any later version.

   scl is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License and a copy of the GNU General Public License along with
   scl. If not, see <http://www.gnu.org/licenses/>.
 */
/* \file scl_tutorial4_control_gc_op.cpp
 *
 *  Created on: Jul 30, 2014
 *
 *  Copyright (C) 2014
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

//scl lib
#include <scl/DataTypes.hpp>
#include <scl/data_structs/SGcModel.hpp>
#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl/dynamics/tao/CDynamicsTao.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/graphics/chai/CGraphicsChai.hpp>
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>
#include <scl/util/DatabaseUtils.hpp>

#include "chai3d.h"

//For timing
#include <sutil/CSystemClock.hpp>

//Eigen 3rd party lib
#include <Eigen/Dense>

//Standard includes (for printing and multi-threading)
#include <iostream>
#include <omp.h>

//Free glut windowing environment
#include <GL/freeglut.h>
#include <math.h>

// Computer Vision
#include "../Kinect/ball_tracking.hpp"

// Puma Client
#include "PumaClient.hpp"

/** A sample application to demonstrate a physics simulation in scl.
 *
 * Moving forward from tutorial 3, we will now control the 6 DOF
 * demo robot (r6bot) with the physics engine running.
 *
 * SCL Modules used:
 * 1. data_structs
 * 2. dynamics (physics)
 * 4. dynamics (control matrices)
 * 4. graphics (chai)
 * */

#define PI 3.14159265
#define kcamX 0
#define kcamY 1
#define kcamZ 2

Eigen::Vector3d Cross_Product(Eigen::Vector3d V1, Eigen::Vector3d V2){
	Eigen::Vector3d V3;
	V3(0) = V1(1)*V2(2) - V1(2)*V2(1);
	V3(1) = V1(2)*V2(0) - V1(0)*V2(2);
	V3(2) = V1(0)*V2(1) - V1(1)*V2(0);
	return (V3);
}

//Eigen::MatrixXd calibrate(Eigen::3d points1[4], Eigen::
Eigen::MatrixXd calibrate(float (&points1)[4][3], float (&points2)[4][3]){  
	Eigen::MatrixXd P(3,3),Q(3,3),M(3,3),T(4,4);
	//P
	//first col
	P(0,0)=points1[1][0]-points1[0][0];
	P(1,0)=points1[2][0]-points1[0][0];
	P(2,0)=points1[3][0]-points1[0][0];
	//second
	P(0,1)=points1[1][1]-points1[0][1];
	P(1,1)=points1[2][1]-points1[0][1];
	P(2,1)=points1[3][1]-points1[0][1];
	//third
	P(0,2)=points1[1][2]-points1[0][2];
	P(1,2)=points1[2][2]-points1[0][2];
	P(2,2)=points1[3][2]-points1[0][2];

	//Q
	//first col
	Q(0,0)=points2[1][0]-points2[0][0];
	Q(1,0)=points2[2][0]-points2[0][0];
	Q(2,0)=points2[3][0]-points2[0][0];
	//second
	Q(0,1)=points2[1][1]-points2[0][1];
	Q(1,1)=points2[2][1]-points2[0][1];
	Q(2,1)=points2[3][1]-points2[0][1];
	//third
	Q(0,2)=points2[1][2]-points2[0][2];
	Q(1,2)=points2[2][2]-points2[0][2];
	Q(2,2)=points2[3][2]-points2[0][2];

	M=P.inverse()*Q;

	Eigen::MatrixXd M_i=M.inverse();
	Eigen::MatrixXd Ox(1,3);
	Eigen::MatrixXd a1(1,3);
	Eigen::MatrixXd a2(1,3);
	
	a1(0,0)=points1[0][0];
	a1(0,1)=points1[0][1];
	a1(0,2)=points1[0][2];
	
	a2(0,0)=points2[0][0];
	a2(0,1)=points2[0][1];
	a2(0,2)=points2[0][2];
    Eigen::MatrixXd a3=a1*M;
    cout<<"a1*M"<<endl;
    for (int i=0;i<3;i++){
        cout<<a3(0,i)<<endl;
        }
        cout<<"a2"<<endl;
    for (int i=0;i<3;i++){
        cout<<a2(0,i)<<endl;
        }
	Ox=a2-(a1*M);
	cout<<"OX"<<endl;
    for (int i=0;i<3;i++){
        cout<<Ox(0,i)<<endl;
        }
	for (unsigned int i=0;i<3;i++){
		for (unsigned int j=0;j<3;j++){
			T(i,j)=M_i(i,j);
		}
	}

	for (unsigned int i=0;i<3;i++){
		T(i,3)=Ox(0,i);
	}

	for (unsigned int i=0;i<3;i++){
		T(3,i)=0;
	}

	T(3,3)=1;

	return T;
}

int main(int argc, char** argv)
{
	std::cout<<"\n***************************************\n";
	std::cout<<"Standard Control Library Tutorial #4";
	std::cout<<"\n***************************************\n";

	scl::SRobotParsed rds;     //Robot data structure....
	scl::SGraphicsParsed rgr;  //Robot graphics data structure...
	scl::SGcModel rgcm;        //Robot data structure with dynamic quantities...
	scl::SRobotIO rio;         //I/O data structure
	scl::CGraphicsChai rchai;  //Chai interface (updates graphics rendering tree etc.)
	scl::CDynamicsScl dyn_scl; //Robot kinematics and dynamics computation object...
	scl::CDynamicsTao dyn_tao; //Robot physics integrator
	scl::CParserScl p;         //This time, we'll parse the tree from a file...
	sutil::CSystemClock::start();

	/******************************Load Robot Specification************************************/
	//We will use a slightly more complex xml spec than the first few tutorials
	bool flag = p.readRobotFromFile("../../specs/Puma/PumaCfg.xml","../../specs/","PumaBot",rds);
	flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
	flag = flag && dyn_tao.init(rds);         //Set up integrator object
	flag = flag && dyn_scl.init(rds);         //Set up kinematics and dynamics object
	flag = flag && rio.init(rds.name_,rds.dof_);

	for(unsigned int i=0;i<rds.dof_;i++){ rio.sensors_.q_(i) = rds.rb_tree_.at(i)->joint_default_pos_; }

	if(false == flag){ return 1; }            //Error check.

	/******************************ChaiGlut Graphics************************************/
	glutInit(&argc, argv); // We will use glut for the window pane (not the graphics).

	flag = p.readGraphicsFromFile("../../specs/Puma/PumaCfg.xml","PumaBotStdView",rgr);
	flag = flag && rchai.initGraphics(&rgr);
	flag = flag && rchai.addRobotToRender(&rds,&rio);
	flag = flag && scl_chai_glut_interface::initializeGlutForChai(&rgr, &rchai);
	if(false==flag) { std::cout<<"\nCouldn't initialize chai graphics\n"; return 1; }

	/******************************Simulation************************************/
	// Now let us integrate the model for a variety of timesteps and see energy stability
	std::cout<<"\nIntegrating the Puma's physics. \nWill test two different controllers.\n Press (x) to exit at anytime.\n";

	//Setting simulation time.
	long iter = 0, n_iters=20000000; double dt=0.0001;

	//# of threads.
	omp_set_num_threads(4);

	int thread_id; double tstart, tcurr; flag = false;

	const Eigen::Vector3d hpos(-0.24,0,0.0); //control position of off-set point wrt. hand
	Eigen::MatrixXd Jx;
	double zi = 0.3;
	double ti = 0;
	double delta_T = 2; //2sec

	double param, theta;
	const Eigen::Vector3d offset(0,-0.5,zi+0.1); //control position of off-set of Kinect

	Eigen::Vector3d x_init;
	Eigen::Vector4d X_ball, V_ball, a_ball, X_ball_init, V_ball_init, V_ball_final;

	//Difference between two coordinate, which is used in Orientation control
	Eigen::Vector3d delta_PHI;

	//Position Control
	Eigen::Vector3d x, x_des, x_des_exp;

	//Current rotation matrix.
	Eigen::Vector3d R1_curr, R2_curr, R3_curr;

	//R2_des and R3_des are used to calculate the desired direction of x-axis at end-effector.
	Eigen::Vector3d R1_des, R2_des, R3_des;

	Eigen::Vector3d dx_v;
	Eigen::Vector3d dx_w;

	//Position Control
	Eigen::Vector3d F_posi, F_posi_decoupled;
	//Orientation Control
	Eigen::Vector3d F_orient, F_orient_decoupled;
	//Mass matrix
	Eigen::MatrixXd Lamda_v, Lamda_w;
	//Null space matrix; Control
	Eigen::MatrixXd Np, I, J_v_pseudo;

	// Calibration 
	Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4,4);
	//T = T*offset;
	//T = {
	float points1[4][3], points2[4][3];
	Vec3f ball_pos;
	// Set Calibration Points
	
	float cal_points[4][3] = { {0.5, 0.2, 0.1}, {0.2, 0.5, 0.2}, {0.2, 0.3, 0.4}, {0.2, 0.1, 0.2} };

	// Computer Vision Tracking with Kinect
	BallTracker BT("Purple");
	//x_des << 0,0,zi-0.1;

	I.setZero(7,7);
	for(int i=0; i<7; i++){
		for(int j=0; j<7; j++) if(i==j) I(i,j) = 1;
	}

	//Used to rotate desired y axis to define x axis.
	Eigen::MatrixXd Rotate_90_degree; Rotate_90_degree.setZero(3,3);
	Rotate_90_degree(0,1) = 1; Rotate_90_degree(1,0) = -1; Rotate_90_degree(2,2) = 1;


	/*************Print Transformation matrix of each link of KUKA*************/
	std::string link_name1("end-effector");
	scl::SRigidBodyDyn *rhand = rgcm.rbdyn_tree_.at(link_name1);
	/*
	   if(rhand == NULL){
	   std::cout<<"\nERROR : Could not find the link : "<<link_name1<<" in the rbdyn tree.\n"; return 1;
	   }
	   scl::SRigidBodyDyn *link6 = rgcm.rbdyn_tree_.at("link_6");
	   if(link6 == NULL){
	   std::cout<< "\nERROR : Could not find the link : " << "link_6" << "in the rbdyn tree. \n" ; return 1;
	   }
	   scl::SRigidBodyDyn *link5 = rgcm.rbdyn_tree_.at("link_5");
	   if(link5 == NULL){
	   std::cout<< "\nERROR : Could not find the link : " << "link_5" << "in the rbdyn tree. \n" ; return 1;
	   }
	   scl::SRigidBodyDyn *link4 = rgcm.rbdyn_tree_.at("link_4");
	   if(link4 == NULL){
	   std::cout<< "\nERROR : Could not find the link : " << "link_4" << "in the rbdyn tree. \n" ; return 1;
	   }
	   scl::SRigidBodyDyn *link3 = rgcm.rbdyn_tree_.at("link_3");
	   if(link3 == NULL){
	   std::cout<< "\nERROR : Could not find the link : " << "link_3" << "in the rbdyn tree. \n" ; return 1;
	   }
	   scl::SRigidBodyDyn *link2 = rgcm.rbdyn_tree_.at("link_2");
	   if(link2 == NULL){
	   std::cout<< "\nERROR : Could not find the link : " << "link_2" << "in the rbdyn tree. \n" ; return 1;
	   }
	   scl::SRigidBodyDyn *link1 = rgcm.rbdyn_tree_.at("link_1");
	   if(link1 == NULL){
	   std::cout<< "\nERROR : Could not find the link : " << "link_1" << "in the rbdyn tree. \n" ; return 1;
	   }
	 */

	// Compute kinematic quantities
	//dyn_scl.computeTransformsForAllLinks(rgcm.rbdyn_tree_,rio.sensors_.q_);
	//dyn_scl.computeJacobianWithTransforms(Jx,*rhand,rio.sensors_.q_,hpos);
	//if(false == flag) flag = true;
	/*
	   std::cout<<"Print Transformation Matrix of " << rhand->name_ << ": \n" << rhand->T_o_lnk_.matrix() << std::endl;
	   std::cout<<"Print Transformation Matrix of " << link6->name_ << ": \n" << link6->T_o_lnk_.matrix() << std::endl;
	   std::cout<<"Print Transformation Matrix of " << link5->name_ << ": \n" << link5->T_o_lnk_.matrix() << std::endl;
	   std::cout<<"Print Transformation Matrix of " << link4->name_ << ": \n" << link4->T_o_lnk_.matrix() << std::endl;
	   std::cout<<"Print Transformation Matrix of " << link3->name_ << ": \n" << link3->T_o_lnk_.matrix() << std::endl;
	   std::cout<<"Print Transformation Matrix of " << link2->name_ << ": \n" << link2->T_o_lnk_.matrix() << std::endl;
	   std::cout<<"Print Transformation Matrix of " << link1->name_ << ": \n" << link1->T_o_lnk_.matrix() << std::endl;

	   sutil::CMappedTree<std::string, scl::SRigidBodyDyn>::const_iterator it,ite;
	   for(it = rgcm.rbdyn_tree_.begin(), ite = rgcm.rbdyn_tree_.end(); it!=ite; it++)
	   {
	   std::cout<<"\n\nLink:"<<it->name_<<"\npar_T_o_link\n"<<(it->T_o_lnk_.matrix())<<"\nJcom\n"<<(it->J_com_);
	   std::cout<<"\n\nLink:"<<it->name_<<"\npar_T_link \n"<<(it->T_lnk_.matrix())<<"\nJcom\n"<<(it->J_com_);
	   }
	 */



#pragma omp parallel private(thread_id)
	{
		thread_id = omp_get_thread_num();
		if(thread_id==3)
		{
			// Run Puma Client in Experiment
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
				q_desired_[3] = 45;
				q_desired_[4] = 50;
				q_desired_[5] = 10;
				MoveJGOTO(PumaRobot,q_desired_,q_,dq_);
				cout << "Start Motion done" << endl;
				break;
			}

			/********************************************************/
			// Start Doing our project
			/********************************************************/
			cout << "Start Doing our project 'd'o" << endl;
			while(1)
			{
				cin >> key;
				if(key == 'd')
					break;
			}

			float x_global_track[7];
			//float x_ball[3] = {0.4,-0.4,0};
			//float v_ball[3] = {0,0,0};
			//float x_ball_init[3]; 
			//float v_ball_init[3];
			float x_ball_final[3];
			//float g = -9.81;
			//float ti_init = 0;
			//float zi_puma = 0.2;

			float dx_I[7], x_I[7];
			
			float q_fine_tune[DOF];
			float x_fine_tune[7];
			float x_global[7] = {0.1, 0.6, 0.2, 0.34, 0.35, 0.62, -0.62};
			
			//PumaRobot->getStatus(GET_IPOS,x_global_track);
			//if 
			Move_GOTO(PumaRobot, x_global, x_I, dx_I);

			cout << "Choose the mode : 1. manually (fine tune) 'm' 2. automatically 'a' hitting the ball" << endl;
			cin >> key;
			char mode;
			if(key == 'm'){
				while(1){
					cout << "Giving the resonable position for the robot : quit(q) or tuning(t) \n" << endl;
					cin >> mode;
					if(mode == 't'){
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
					if(mode == 'q'){
						cout << "Stop Doing our project 'q'uit" << endl;
						break;
					}
					cout << "If you want to go to the control task press 'a'" << endl;
					cin >> mode;
					if(mode == 'a'){
						key = 'a';
						break;
					}
				}
			}

			timespec ts;

			if(key == 'a'){
				int i = 0;

				cout << "put the ping pong ball and press 'k' or tuning again? (t) or quit? " << endl;
				cin >> key;
				
				if(key == 't'){
					while(1){
						cout << "Giving the resonable position for the robot : quit(q) or tuning(t) \n" << endl;
						cin >> mode;
						if(mode == 't'){
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
						if(mode == 'q'){
							cout << "quit" << endl;
							break;
						}
					}
				}

				cout << "Calibrate the system? [y/n]?" << endl;
				cin >> key;
				if (key=='y'){

					// Calibration
					for (i = 0; i < 4; i++){
						x_global[0] = cal_points[i][0];
						x_global[1] = cal_points[i][1];
						x_global[2] = cal_points[i][2];

						Move_GOTO(PumaRobot, x_global, x_I, dx_I);
						PumaRobot->getStatus(GET_IPOS, x_global_track);
						points2[i][0]=x_global_track[1];
						points2[i][1]=x_global_track[0];
						points2[i][2]=x_global_track[2];

						cout << "Calibrating camera position. Moving arm to Point "<< i <<". Please place ball on end effector (center of paddle). Press any key when done." << endl;
						cin >> key;

						ball_pos = BT.getPosition(); 
						while (ball_pos == Vec3f(0,0,0)){
							cout<< "No position. Retake picture. Press k."<<endl;
							cin>>key;
							ball_pos = BT.getPosition(); 
						}
						points1[i][0]=ball_pos(kcamX);
						points1[i][1]=ball_pos(kcamY);
						points1[i][2]=ball_pos(kcamZ);
						cout << "Press any key to continue to next position."<<endl;
						cin >> key;

					}
					for (int l=0;l<4;l++){
					    for(int m=0;m<3;m++){
					        cout<<points1[l][m]<<"  "<<points2[l][m]<<endl;
					    }
					}         
					int xxx;
					cin>>xxx;
					
					T = calibrate(points1, points2);
					cout << T << endl;
				}

				i = 0;

				//PumaRobot->getStatus(GET_IPOS,x_global);
				//Move_GOTO_fast(PumaRobot, x_global);



				/*while(1){
				  if(i == 20) break;

				  ts = {0, 200000000};
				  nanosleep(&ts,NULL);

				  x_global[2] = 0.10;
				//x_global[3] = 0.36;
				//x_global[4] = 0.33;
				//x_global[5] = 0.59;
				//x_global[6] = -0.64;

				Move_GOTO_fast(PumaRobot, x_global);

				ts = {0, 100000000};//100ms
				nanosleep(&ts,NULL);

				x_global[2] = 0.00;
				Move_GOTO_fast(PumaRobot, x_global);

				i++;
				}
				i=0;
				}*/
				//x_global = x_fine_tune;
				//x_ball_final = x_fine_tune;
				cout << "Go to the fine tune position? y/n" << endl;

				cin >> key;
				if(key=='y') MoveJGOTO(PumaRobot,q_fine_tune,q_,dq_);
				//	}
				//}
				cout << "Start hitting the ping pong ball? y/n " << endl;

				cin >>key;

				if(key == 'y'){
					PumaRobot->getStatus(GET_IPOS,x_global);
					Move_GOTO_fast(PumaRobot, x_global);


					//cout << "Moving on to catch ball task" << endl;
					//After fine tune everything, so predict the final position of ball first. 
					//Don't forget to check if it's in the reasonable region.

					while(true){
						//cin >> key;

						//This part will be replaced by vision part
						/*
						   cout << "Giving the initial position for the ball" << endl;
						   cout << "x_ball[0] : \n" << endl;
						   cin >> x_ball[0];
						   cout << "x_ball[1] : \n" << endl;
						   cin >> x_ball[1];
						   cout << "x_ball[2] : \n" << endl;
						   cin >> x_ball[2];

						   cout << "Giving the initial velocity for the ball" << endl;
						   cout << "v_ball[0] : \n" << endl;
						   cin >> v_ball[0];
						   cout << "v_ball[1] : \n" << endl;
						   cin >> v_ball[1];
						   cout << "v_ball[2] : \n" << endl;
						   cin >> v_ball[2];
						 */
						/*
						   for(int i=0; i<3; i++){
						   x_ball_init[i] = x_ball[i];
						   v_ball_init[i] = v_ball[i];
						   }

						   ti_init = (v_ball_init[2] + sqrt(v_ball_init[2]*v_ball_init[2] + 2*9.81*(x_ball_init[2]-zi_puma)))/9.8;
						   x_ball_final[0] = x_ball_init[0]+v_ball_init[0]*ti_init;
						   x_ball_final[1] = x_ball_init[1]+v_ball[1]*ti_init;
						   x_ball_final[2] = x_ball_init[2] + v_ball_init[2] - 0.5*9.8*ti_init*ti_init;
						 */

						//
						//Define y axis : Rotate 90 degrees from y axis
						cout << "From sim: " << x_des(0) << "," << x_des(1) << "," << x_des(2) <<"      ";

						//float dist = (x_des_exp - Rotate_90_degree * x_des).norm();
						float dist = (x_des_exp -  x_des).norm();
						//x_des_exp = Rotate_90_degree * x_des;
						x_des_exp = x_des;
						x_ball_final[0] = x_des_exp(0);
						x_ball_final[1] = x_des_exp(1);
						x_ball_final[2] = x_des_exp(2);
						cout << "Moving to position: " << x_des_exp(0) << "," << x_des_exp(1) << "," << x_des_exp(2) << endl;

						if(x_ball_final[0] > 0 && x_ball_final[1] < 0){
							if((x_ball_final[0]*x_ball_final[0] + x_ball_final[1]*x_ball_final[1]) < 0.72 && (x_ball_final[0]*x_ball_final[0] + x_ball_final[1]*x_ball_final[1]) > 0.1){
								//if (dist > 0.15){ // distance

								//x_des_exp = x_des;


								//Move_GOTO(PumaRobot, x_ball_final, x_I, dx_I);
								Move_GOTO_fast(PumaRobot, x_ball_final);
								PumaRobot->getStatus(GET_IPOS,x_global_track);
								cout << "x tracking : " << x_global_track[0] << " " << x_global_track[1] << " " << x_global_track[2] << " " << endl;	
								//}
							}
						}
						else{
							//Out of range the robot can reach.

						}



					}
				}
}

// float the before disconnecting
PumaRobot->_float();

//Disconnect Robot

//Sleep(2000);
//PumaRobot->~RobotCom();

}
else if(thread_id==2)
{
	// Start Tracking ball with Computer Vision
	BT.run();
}
else if(thread_id==1) //Simulate physics and update the rio data structure..
{
	/******************************Draw a ping pong ball************************************/
	chai3d::cShapeSphere * ping_pong = new chai3d::cShapeSphere(0.05); // create sphere
	rchai.getChaiData()->chai_world_->addChild(ping_pong); // insert object into world

	/****************************Set Initial Position for the ball*****************************/
	//*
	Vec3f position = BT.getPosition(); // x, z, y
	Vec3f velocity = BT.getVelocity();
	X_ball(0) = offset(0) + position[0]; X_ball(1) = offset(1) + position[2]; X_ball(2) = offset(2) + position[1];
	/*X_ball(0) = position[kcamX]; 
	X_ball(1) = position[kcamY]; 
	X_ball(2) = position[kcamZ]; 
	X_ball(3) = 1;
	V_ball(0) = velocity[kcamX]; 
	V_ball(1) = velocity[kcamY]; 
	V_ball(2) = velocity[kcamZ]; 
	V_ball(3) = 1;

	X_ball = T*X_ball;
	V_ball = T*V_ball;
	*/	
/*/
	  X_ball(0) = 0; X_ball(1) = 0.2; X_ball(2) = 5;
	  V_ball(0) = 0; V_ball(1) = 0; V_ball(2) = 0;
	//*/
	a_ball(0) = 0; a_ball(1) = 0; a_ball(2) = -9.81;

	X_ball_init = X_ball;
	V_ball_init = V_ball;
	V_ball_final = V_ball;

	//Set location for the ball.
	ping_pong->setLocalPos(X_ball(0), X_ball(1), X_ball(2));

	// Controller 2 : Operational space controller
	std::cout<<"\n\n***************************************************************"
		<<"\n Starting op space (task coordinate) controller..."
		<<"\n This will make a trajectory of ping pong ball"
		<<"\n***************************************************************";
	tstart = sutil::CSystemClock::getSimTime();
	iter = 0;

	while((true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running) && (iter<=n_iters))
	{
		tcurr = sutil::CSystemClock::getSimTime();

		//*
		position = BT.getPosition(); // x, z, y
		velocity = BT.getVelocity();
		if ( position != Vec3f(0,0,0) ) { // Update if valid
			X_ball(0) = offset(0) + position[0]; X_ball(1) = offset(1) + position[2]; X_ball(2) = offset(2) + position[1];
	        /*X_ball(0) = position[kcamX]; 
	        X_ball(1) = position[kcamY]; 
	        X_ball(2) = position[kcamZ]; 
	        X_ball(3) = 1;
	        V_ball(0) = velocity[kcamX]; 
	        V_ball(1) = velocity[kcamY]; 
	        V_ball(2) = velocity[kcamZ]; 
	        V_ball(3) = 1;

			X_ball = T*X_ball;
			V_ball = T*V_ball;
		*/		
		}
		/*/
		//Motion of ball
		X_ball(0) = X_ball(0) + V_ball(0)*dt;
		X_ball(1) = X_ball(1) + V_ball(1)*dt;
		X_ball(2) = X_ball(2) + V_ball(2)*dt;
		V_ball(2) = V_ball(2) - 9.8*dt; 
		//*/

		ping_pong->setLocalPos(X_ball(0), X_ball(1), X_ball(2));
		X_ball_init = X_ball;
		V_ball_init = V_ball;

		ti = (V_ball_init(2) + sqrt(V_ball_init(2)*V_ball_init(2) + 2*9.8*(X_ball_init(2)-zi)))/9.8;

		/* 
		   std::cout << x_des(0)<<","<<x_des(1)<<","<<x_des(2)<< std::endl;
		   std::cout << X_ball(0)<<","<<X_ball(1)<<","<<X_ball(2)<< std::endl;
		   std::cout << V_ball(0)<<","<<V_ball(1)<<","<<V_ball(2)<< std::endl;
		   std::cout << ti<< std::endl;
		 */

		// Compute kinematic quantities
		dyn_scl.computeTransformsForAllLinks(rgcm.rbdyn_tree_,rio.sensors_.q_);
		dyn_scl.computeJacobianWithTransforms(Jx,*rhand,rio.sensors_.q_,hpos);
		if(false == flag) { x_init = rhand->T_o_lnk_ * hpos; flag = true; }

		//Get current X_global.
		x = rhand->T_o_lnk_ * hpos;

		//Angular Velocity
		dx_v = Jx.block(0,0,3,rio.dof_) * rio.sensors_.dq_;

		dx_w = Jx.block(3,0,3,rio.dof_) * rio.sensors_.dq_;

		/********Get current rotation matrix*******/
		R1_curr << rhand->T_o_lnk_(0,0),rhand->T_o_lnk_(1,0),rhand->T_o_lnk_(2,0);
		R2_curr << rhand->T_o_lnk_(0,1),rhand->T_o_lnk_(1,1),rhand->T_o_lnk_(2,1);
		R3_curr << rhand->T_o_lnk_(0,2),rhand->T_o_lnk_(1,2),rhand->T_o_lnk_(2,2);

		///*
		if(0.1 > abs(ti)){
			if((X_ball(2) < (zi+0.2)) && (V_ball(2) < 0)){
				x_des << X_ball(0)+V_ball(0)*dt, X_ball(1)+V_ball(1)*dt, zi+0.05;
				R1_des << 0,0,1;


				//if((X_ball(2) < zi) && (V_ball(2) < 0)) {
				//V_ball(2) = -V_ball(2) + ((rand()%100)-50)/1000.0; 
				//V_ball(0) = ((rand()%100)-50)/1000.0; V_ball(1) = ((rand()%100)-50)/1000.0;
				//V_ball(0) = 0; V_ball(1) = 0;
				//}

			}

			else if((X_ball(2) > zi) && (V_ball(2) >= 0)){
				x_des << X_ball(0)+V_ball(0)*dt, X_ball(1)+V_ball(1)*dt, zi-0.01;
				R1_des << 0,0,1;
			}
		} else if(0 < ti){
			//*/

			//else{
			//Move the end effector to be under the predict (x,y) position of ball.
			x_des << X_ball_init(0)+V_ball_init(0)*ti, X_ball_init(1)+V_ball_init(1)*ti, zi-0.01;

			V_ball_final(2) = V_ball_init(2) -9.8*ti;


			/********Set desired rotation matrix******/
			/*
			//Calculate theta : difference of angle between global z-axis and end-effector z-axis
			double dot_result = V_ball_final(2);
			double length_result = sqrt(V_ball_final(0)*V_ball_final(0) + V_ball_final(1)*V_ball_final(1) + V_ball_final(2)*V_ball_final(2));

			param = dot_result / length_result;
			theta =90 - 0.5 * acos (param) * 180.0 / PI;

			//Define z axis : point to the ball comming direction
			R3_des(0) = -V_ball_final(0);
			R3_des(1) = -V_ball_final(1);
			R3_des(2) = 0;

			//Define y axis : Rotate 90 degrees from y axis
			R2_des = Rotate_90_degree * R3_des;

			double result;
			result = tan ( theta * PI / 180.0 );
			R3_des(2) = -result * sqrt(R2_des(0)*R2_des(0) + R2_des(1)*R2_des(1));
			R2_des(2) = R3_des(2);

			double length_z = sqrt(R3_des(0)*R3_des(0) + R3_des(1)*R3_des(1) + R3_des(2)*R3_des(2));
			double length_y = sqrt(R2_des(0)*R2_des(0) + R2_des(1)*R2_des(1) + R2_des(2)*R2_des(2));

			R3_des(0) = R3_des(0) / length_z; R3_des(1) = R3_des(1) / length_z; R3_des(2) = R3_des(2) / length_z;
			R2_des(0) = R2_des(0) / length_y; R2_des(1) = R2_des(1) / length_y; R2_des(2) = R2_des(2) / length_y;

			//Using cross product to find x-axis
			R1_des = Cross_Product(R2_des, R3_des);
			//TODO
			R1_des << 1,0,0;
			 */
			///*
		} else {
			//std::cout<<"Time of impact is negative"<<endl;
		}
		// */

		R2_des << 0,0,1;




		/*************************Position Control***********************/
		Lamda_v = Jx.block(0,0,3,rio.dof_)*rgcm.M_gc_.inverse()*Jx.block(0,0,3,rio.dof_).transpose();

		Lamda_v = Lamda_v.inverse();

		F_posi = (1600 * (x_des - x) - 80*dx_v);

		F_posi_decoupled = Lamda_v * F_posi;

		/************************Orientation Control********************/
		Lamda_w = Jx.block(3,0,3,rio.dof_)*rgcm.M_gc_.inverse()*Jx.block(3,0,3,rio.dof_).transpose();

		Lamda_w = Lamda_w.inverse();

		//Get delta_PHI
		delta_PHI = -0.5*(Cross_Product(R1_curr, R1_curr) + Cross_Product(R2_des, R2_curr) + Cross_Product(R3_curr, R3_curr));

		F_orient = (1600 * delta_PHI - 80*dx_w);

		F_orient_decoupled = Lamda_w * F_orient;

		/*****************Null Space Matrix*****************************/
		J_v_pseudo = rgcm.M_gc_.inverse() * Jx.block(0,0,3,rio.dof_).transpose() * Lamda_v;
		Np = I - Jx.block(0,0,3,rio.dof_).transpose() * J_v_pseudo.transpose();

		/******************Calculation of Joint force*******************/
		rio.actuators_.force_gc_commanded_ =
			Jx.block(0,0,3,rio.dof_).transpose() * F_posi_decoupled - 20*rio.sensors_.dq_
			+
			Np * (Jx.block(3,0,3,rio.dof_).transpose() * F_orient_decoupled) - 20*rio.sensors_.dq_;





		// Integrate the dynamics
		dyn_tao.integrate(rio,dt);

		const timespec ts = {0, dt*1e9}; //0.05ms
		nanosleep(&ts,NULL);

		sutil::CSystemClock::tick(dt);
		iter++;
		}
		//Then terminate
		scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running = false;
	}
	else  //Read the rio data structure and updated rendererd robot..
		while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
		{
			glutMainLoopEvent();
			const timespec ts = {0, 15000000};/*15ms*/
			nanosleep(&ts,NULL);
		}
}

/******************************Exit Gracefully************************************/
std::cout<<"\n\nExecuted Successfully";
std::cout<<"\n**********************************\n"<<std::flush;

return 0;
}
