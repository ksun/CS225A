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
#include <math.h>

//Freeglut windowing environment
#include <GL/freeglut.h>

// Computer Vision
#include "../Kinect/ball_tracking.hpp"

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
	bool flag = p.readRobotFromFile("../../specs/Kuka/KukaCfg_orig.xml","../../specs/","KukaBot",rds);
	flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
	flag = flag && dyn_tao.init(rds);         //Set up integrator object
	flag = flag && dyn_scl.init(rds);         //Set up kinematics and dynamics object
	flag = flag && rio.init(rds.name_,rds.dof_);
	for(unsigned int i=0;i<rds.dof_;++i){ rio.sensors_.q_(i) = rds.rb_tree_.at(i)->joint_default_pos_; }
	if(false == flag){ return 1; }            //Error check.

	/******************************ChaiGlut Graphics************************************/
	glutInit(&argc, argv); // We will use glut for the window pane (not the graphics).

	flag = p.readGraphicsFromFile("../../specs/Kuka/KukaCfg_orig.xml","KukaBotStdView",rgr);
	flag = flag && rchai.initGraphics(&rgr);
	flag = flag && rchai.addRobotToRender(&rds,&rio);
	flag = flag && scl_chai_glut_interface::initializeGlutForChai(&rgr, &rchai);
	if(false==flag) { std::cout<<"\nCouldn't initialize chai graphics\n"; return 1; }

	/******************************Simulation************************************/
	// Now let us integrate the model for a variety of timesteps and see energy stability
	std::cout<<"\nIntegrating the r6bot's physics. \nWill test two different controllers.\n Press (x) to exit at anytime.";
	long iter = 0, n_iters=200; double dt=0.0001;

	omp_set_num_threads(3);
	int thread_id; double tstart, tcurr; flag = false;
	const Eigen::Vector3d hpos0(0.001,0,0); //control position of op-point wrt. hand
	const Eigen::Vector3d hpos1(0.001,.2,0); //control position of op-point wrt. hand
	Eigen::MatrixXd Jx0, Jx1;
	Eigen::Vector3d x0, x0_des(0,0,0), x_init0, dx0;
	Eigen::Vector3d x1, x1_des=x0_des+(hpos1-hpos0), x_init1, dx1;
	scl::SRigidBodyDyn *rhand = rgcm.rbdyn_tree_.at("end-effector");

	// Computer Vision Tracking with Kinect
	const Vec3f offset(0.25,5,.25); //control position of off-set of Kinect
	BallTracker BT("Green");

#pragma omp parallel private(thread_id)
	{
		thread_id = omp_get_thread_num();
		if(thread_id==2)
		{
			// Start Tracking ball with Computer Vision
			BT.run();
		}
		else if(thread_id==1) //Simulate physics and update the rio data structure..
		{
			/******************************Draw a ping pong ball************************************/
			chai3d::cShapeSphere * ping_pong = new chai3d::cShapeSphere(0.05); // create sphere
			rchai.getChaiData()->chai_world_->addChild(ping_pong); // insert object into world


			// Controller 2 : Operational space controller
			std::cout<<"\n\n***************************************************************"
				<<"\n Starting op space (task coordinate) controller..."
				<<"\n This will move the hand in a circle. x =sin(t), y=cos(t)."
				<<"\n***************************************************************";

			tstart = sutil::CSystemClock::getSimTime(); iter = 0;
			Eigen::Vector3d ball_force,ball_pos,ball_vel,ball_accel,paddle_orientation,x_init,ball_vel_store;
			Eigen::Vector3d ball_pos_int (.25, .25, 10);
			Eigen::Vector3d ball_vel_int (0, 0, 0);
			Eigen::Vector3d gravity  (0,0,-9.8),ez(0,0,1);
			Eigen::Vector3d ball_force_est,ball_pos_est, ball_vel_est,ball_accel_est,paddle_orientation_des;
			bool hitting = false, impact = false,steps=false;
			float ball_pos_threshold = .1,radius=.02,rho=1,Cd=1,mass=.0027,impact_time=(10000*dt),Delta_T=.0001;
			int est_iter = 0,iters_impact=0;
			ball_pos=ball_pos_int;
			ball_vel=ball_vel_int;
			Vec3f position = BT.getPosition(); // x, z, y
			Vec3f velocity = BT.getVelocity();
			ball_pos(0) = position[0];  ball_pos(1) = position[2];  ball_pos(2) = position[1];
			ball_vel(0) = velocity[0];ball_vel(1) = velocity[2]; ball_vel(2) = velocity[1];
			
			ping_pong->setLocalPos(ball_pos(0), ball_pos(1), ball_pos(2));

			while( iter<200000 && true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
			{
				tcurr = sutil::CSystemClock::getSimTime();      
				position = BT.getPosition(); // x, z, y
				velocity = BT.getVelocity();
				if ( position != Vec3f(0,0,0) ) { // Update if valid

					ball_pos(0) = position[0]; ball_pos(1) = position[2]; ball_pos(2) = position[1];
					ball_vel(0) = velocity[0]; ball_vel(1) = velocity[2]; ball_vel(2) = velocity[1];
					}
					ping_pong->setLocalPos(ball_pos(0), ball_pos(1), ball_pos(2));

					int xxxx;
					std::cout<<"\n";
					if (iter % 100 == 0){
						// std::cin>>xxxx;
					}
					//std::cout<<"\n"<<"x is "<<x0<<" \n x des is"<<x0_des;
					//std::cout<<"\n x1 is "<<x1<<"\n x1des is "<<x1_des;


					paddle_orientation=x1-x0;
					if(iter>=iters_impact){
						if(ball_pos(2)<x0(2)){ //checks for impact
							std::cout<<"\n z pos end effector"<<x0(2);
							Eigen::MatrixXd delme = ball_vel.transpose()*paddle_orientation;
							double xx = 2*delme(0,0);
							xx = xx/paddle_orientation.norm();
							ball_vel -= xx*paddle_orientation.normalized();
							ball_vel = 1.05*ball_vel.normalized();
							ball_vel_store=ball_vel;
							//ball_vel_store(2)=ball_vel_store(2);
							std::cout<<"\n ball vel"<<ball_vel.transpose();
							int xxx;
							x0_des(2)=0;
							impact=true;
							hitting=false;
							iters_impact=iter+7000;
							impact_time=(iter+1000000)*dt;
						}else{ //if no impact
							if(hitting){       //check to see if hitting
								x0_des(2)=ball_pos_threshold+.1;

							}else if(iter*dt>impact_time-Delta_T){ //check to see it should be hitting
								hitting=true;
								std::cout<<"\n hitting";
							}else{ //if no impact and not hitting esitmate trejectory
								ball_pos_est=ball_pos; //intialize for estimation
								ball_vel_est=ball_vel;
								est_iter = 0;
								std::cout<<"\n**********************EST*********************";
								while(ball_pos_est(2) > ball_pos_threshold){//estimated trajectory
									double xx = .5*rho*3.14159*radius*radius*Cd*ball_vel_est.norm();
									ball_force_est = -1* xx*ball_vel_est;
									ball_accel_est=gravity+(ball_force_est/mass);
									ball_vel_est=ball_vel_est+ball_accel_est*dt;
									ball_pos_est=ball_pos_est+ball_vel_est*dt;
									est_iter++;
								}
								impact_time=(iter+est_iter)*dt;
								x0_des(0) = ball_pos_est(0); //crossing location in z plane
								x0_des(1) = ball_pos_est(1);

							}
							//computing ball forces since no impact
							double xxxx=.5*rho*3.14159*radius*radius*Cd*ball_vel.norm();
							//  std::cout<<"xxxx is "<<xxxx;
							ball_force=-1*xxxx*ball_vel;
							ball_accel=gravity+ball_force/mass;
							// ball_vel=ball_vel+ball_accel*dt;

						}
					}else{
						x0_des(2)=0;
						//ball_vel=ball_vel_store;
						std::cout<<"\n x des "<<x0_des.transpose()<<" x1 des"<<x1_des.transpose();


					}



					paddle_orientation_des=(ez-ball_vel_est)/sqrt(2*ball_vel_est.norm()-ball_vel_est.transpose()*ez);
					paddle_orientation_des=paddle_orientation_des.normalized()*.2;

					x1_des=x0_des+paddle_orientation_des;
//if(ball_vel(2) == 0) { x1_des = x_des};


					// Integrate the dynamics
					dyn_tao.integrate(rio,dt); iter++;


					//ball_pos=ball_pos+ball_vel*dt;


					std::cout<<"\n ball pos "<<ball_pos.transpose()<<" ball vel "<<ball_vel.transpose();
					std::cout<<"\n x des "<<x0_des.transpose()<<" x1 des"<<x1_des.transpose();

					// Compute kinematic quantities
					dyn_scl.computeTransformsForAllLinks(rgcm.rbdyn_tree_,rio.sensors_.q_);

					dyn_scl.computeJacobianWithTransforms(Jx0,*rhand,rio.sensors_.q_,hpos0);
					dyn_scl.computeJacobianWithTransforms(Jx1,*rhand,rio.sensors_.q_,hpos1);


					if(false == flag) { x_init = rhand->T_o_lnk_ * hpos0; flag = true; }

					x0 = rhand->T_o_lnk_ * hpos0;
					x1 = rhand->T_o_lnk_ * hpos1;

					dx0 = Jx0.block(0,0,3,rio.dof_) * rio.sensors_.dq_;
					dx1 = Jx1.block(0,0,3,rio.dof_) * rio.sensors_.dq_;

					rio.actuators_.force_gc_commanded_ = Jx0.block(0,0,3,rio.dof_).transpose() * (3000*(x0_des-x0) - 10 * dx0);
					rio.actuators_.force_gc_commanded_ += Jx1.block(0,0,3,rio.dof_).transpose() * (3000*(x1_des-x1) - 50 * dx1) - 20*rio.sensors_.dq_;

					if (!(rio.actuators_.force_gc_commanded_(0)>0) and !(rio.actuators_.force_gc_commanded_(0) < 0)){
						std::cout<<x0_des<<"   "<<x1_des;
						std::cout<<"\n"<<rio.actuators_.force_gc_commanded_.transpose();
						int xx;
						//std::cin>>xx;
					}

					// Integrate the dynamics
					dyn_tao.integrate(rio,dt); iter++; const timespec ts = {0, 5000};/*.05ms*/
					nanosleep(&ts,NULL);
					sutil::CSystemClock::tick(dt);
					if(iter % 1000 == 0)
					{ //std::cout<<"\n"<<rio.actuators_.force_gc_commanded_<<" "<<(x0_des-x0).norm();
						//std::cout<<"\n"<<rio.actuators_.force_gc_commanded_.transpose();
					}
				}
				//Then terminate
				scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running = false;
			}
		else  //Read the rio data structure and updated rendererd robot..
			while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
			{ glutMainLoopEvent(); const timespec ts = {0, 15000000};/*15ms*/
				nanosleep(&ts,NULL); }
	}


	/******************************Exit Gracefully************************************/
	std::cout<<"\n\nExecuted Successfully";
	std::cout<<"\n**********************************\n"<<std::flush;

	return 0;
}
