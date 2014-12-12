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
  bool flag = p.readRobotFromFile("../../specs/Kuka/KukaCfg.xml","../../specs/","KukaBot",rds);
  flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
  flag = flag && dyn_tao.init(rds);         //Set up integrator object
  flag = flag && dyn_scl.init(rds);         //Set up kinematics and dynamics object
  flag = flag && rio.init(rds.name_,rds.dof_);
  for(unsigned int i=0;i<rds.dof_;++i){ rio.sensors_.q_(i) = rds.rb_tree_.at(i)->joint_default_pos_; }
  if(false == flag){ return 1; }            //Error check.

  /******************************ChaiGlut Graphics************************************/
  glutInit(&argc, argv); // We will use glut for the window pane (not the graphics).

  flag = p.readGraphicsFromFile("../../specs/Kuka/KukaCfg.xml","KukaBotStdView",rgr);
  flag = flag && rchai.initGraphics(&rgr);
  flag = flag && rchai.addRobotToRender(&rds,&rio);
  flag = flag && scl_chai_glut_interface::initializeGlutForChai(&rgr, &rchai);
  if(false==flag) { std::cout<<"\nCouldn't initialize chai graphics\n"; return 1; }

  /******************************Simulation************************************/
  // Now let us integrate the model for a variety of timesteps and see energy stability
  std::cout<<"\nIntegrating the r6bot's physics. \nWill test two different controllers.\n Press (x) to exit at anytime.";
  long iter = 0, n_iters=200; double dt=0.0001;

  omp_set_num_threads(2);
  int thread_id; double tstart, tcurr; flag = false;
  const Eigen::Vector3d hpos0(0,1,0); //control position of op-point wrt. hand
  const Eigen::Vector3d hpos1(0,1,1); //control position of op-point wrt. hand
  Eigen::MatrixXd Jx0, Jx1;
  Eigen::Vector3d x0, x0_des=(0,0,0), x_init0, dx0;
  Eigen::Vector3d x1, x1_des1=x0_des+(hpos1-hpos0), x_init1, dx1;
  scl::SRigidBodyDyn *rhand = rgcm.rbdyn_tree_.at("hand");

#pragma omp parallel private(thread_id)
  {
    thread_id = omp_get_thread_num();
    if(thread_id==1) //Simulate physics and update the rio data structure..
    {
      // Controller 1 : gc controller
      std::cout<<"\n\n***************************************************************"
          <<"\n Starting joint space (generalized coordinate) controller..."
          <<"\n This will move the joints in a sine wave"
          <<"\n***************************************************************";
      tstart = sutil::CSystemClock::getSimTime();
      sutil::CSystemClock::tick(dt);
      while(iter < n_iters && true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      {
        tcurr = sutil::CSystemClock::getSysTime();
        // Controller : fgc = kp * (sin(t) - q) + kv(0 - dq)
        // Set the desired positions so that the joints move in a sine wave
        rio.actuators_.force_gc_commanded_ = 50* (sin(tcurr-tstart) - rio.sensors_.q_.array()) - 20 *       rio.sensors_.dq_.array();
        dyn_tao.integrate(rio,dt);
        iter++;
        const timespec ts = {0, 5000};/*.05ms*/
        nanosleep(&ts,NULL);

        if(iter % 1000 == 0){
           // std::cout<<"\n"<<(sin(tcurr-tstart) - rio.sensors_.q_.array()).transpose();
        //std::cout<<"\n"<<rio.actuators_.force_gc_commanded_.transpose();
            }
      }
     sleep(1);

      // Controller 2 : Operational space controller
      std::cout<<"\n\n***************************************************************"
          <<"\n Starting op space (task coordinate) controller..."
          <<"\n This will move the hand in a circle. x =sin(t), y=cos(t)."
          <<"\n***************************************************************";

      tstart = sutil::CSystemClock::getSimTime(); iter = 0;
	  Eigen::Vector3d ball_forces,ball_pos,ball_vel,ball_accel,paddle_orientation;
      Eigen::Vector3d ball_pos_int = (1, 1, 10);
      Eigen::Vector3d ball_vel_int =(0, 0, 0);
      Eigen::Vector3d gravity = (0,0,-9.8),ez=(0,0,1);
      Eigen::Vector3d ball_force_est,ball_pos_est, ball_vel_est,ball_accel_est,orientation;
      bool hitting = false;
      float ball_pos_threshold = 2,radius=.025,rho=1,Cd=.1,mass=1,impact_time=(10000*dt),Delta_T=.01;
      int est_iter = 0;
      ball_pos=ball_pos_int;
      ball_vel=ball_vel_int;

      while( iter<200000 && true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      {
        tcurr = sutil::CSystemClock::getSimTime();


		paddle_orientation=x1-x0;


		if(ball_pos(2)<x0(2)){ //checks for impact
			ball_vel=1.2*ball_vel-2(ball_vel.transpose()*paddle_orientation.normalized())*paddel_orientation.normalized();
			rtask_hand->x0_des(2)=0;
			hitting=false;
			impact_time=(iter+10000)*dt;

		}else{ //if no impact
			if(hitting){       //check to see if hitting          
				rtask_hand->x0_des(2)=100;

		}else if(iter*dt>imapct_time-Delta_T){ //check to see it should be hitting
			hitting=true;  

		}else{ //if no impact and not hitting esitmate trejectory
			ball_pos_est=ball_pos; //intialize for estimation
			ball_vel_est=ball_vel;
			est_iter = 0;

			while(ball_pos_est(2) > ball_pos_threshold){//estimated trajectory
				ball_force_est=gravity-.5*rho*3.14159*radius*radius*Cd*ball_vel_est.squaredNorm()*ball_vel_est.Norm();
				ball_accel_est=ball_force_est/mass;
				ball_vel_est=ball_vel_est+ball_accel_est*dt;
				ball_pos_est=ball_pos_est+ball_vel_est*dt;
				est_iter++;
									     							  
			}							  
			impact_time=(iter+est_iter)*dt;
			x0_des(0) = ball_pos_est(0); //crossing location in z plane
			x0_des(1) = ball_pos_est(1);
	      
		}
	    //computing ball forces since no impact
			ball_force=gravity-.5*rho*3.14159*radius*radius*Cd*ball_vel.squaredNorm()*ball_vel.Norm();
			ball_accel=ball_force/mass;
			ball_vel=ball_vel+ball_accel*dt;
				
		}
		

		

		orientation=(ez-ball_vel)/sqrt(2*ball_vel.norm()-ball_vel.transpose()*(0,0,1));

		x1_des=x0_des+orientation.normalized();

        // Compute control forces (note that these directly have access to the io data ds).
        rctr.computeDynamics();
        rctr.computeControlForces();

        // Integrate the dynamics
        dyn_tao.integrate(rio,dt); iter++;
		ball_pos=ball_pos+ball_vel*dt;

        // Compute kinematic quantities
        dyn_scl.computeTransformsForAllLinks(rgcm.rbdyn_tree_,rio.sensors_.q_);

        dyn_scl.computeJacobianWithTransforms(Jx0,*rhand,rio.sensors_.q_,hpos0);
        dyn_scl.computeJacobianWithTransforms(Jx1,*rhand,rio.sensors_.q_,hpos1);

        
		if(false == flag) { x_init = rhand->T_o_lnk_ * hpos0; flag = true; }

        x0 = rhand->T_o_lnk_ * hpos0; 
        x1 = rhand->T_o_lnk_ * hpos1; 

        dx0 = Jx0.block(0,0,3,rio.dof_) * rio.sensors_.dq_;
		dx1 = Jx1.block(0,0,3,rio.dof_) * rio.sensors_.dq_;

        rio.actuators_.force_gc_commanded_ = Jx0.block(0,0,3,rio.dof_).transpose() * (1000*(x0_des-x0) - 10 * dx0) - 200*rio.sensors_.dq_;
        rio.actuators_.force_gc_commanded_ += Jx1.block(0,0,3,rio.dof_).transpose() * (1000*(x1_des-x1) - 10 * dx1) - 200*rio.sensors_.dq_;

        // Integrate the dynamics
        dyn_tao.integrate(rio,dt); iter++; const timespec ts = {0, 5000};/*.05ms*/
        nanosleep(&ts,NULL);
        sutil::CSystemClock::tick(dt);
        if(iter % 1000 == 0)
        { std::cout<<"\n"<<(x0_des-x0).transpose()<<" "<<(x0_des-x0).norm();
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
