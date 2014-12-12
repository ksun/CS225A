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

Eigen::Vector3d Cross_Product(Eigen::Vector3d V1, Eigen::Vector3d V2){
  Eigen::Vector3d V3;
  V3(0) = V1(1)*V2(2) - V1(2)*V2(1);
  V3(1) = V1(2)*V2(0) - V1(0)*V2(2);
  V3(2) = V1(0)*V2(1) - V1(1)*V2(0);
  return (V3);
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

  char mode = 'n';
  std::cout << "Select a robot Puma (p) / Kuka (k) : \n";
  std::cin >> mode;

  /******************************Load Robot Specification************************************/
  //We will use a slightly more complex xml spec than the first few tutorials

  bool flag = false;
  if(mode == 'k')
    flag = p.readRobotFromFile("../../../../specs/Kuka/KukaCfg.xml","../../../specs/","KukaBot",rds);
  if(mode == 'p')
    flag = p.readRobotFromFile("../../../../specs/Puma/PumaCfg.xml","../../../specs/","PumaBot",rds);
  flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
  flag = flag && dyn_tao.init(rds);         //Set up integrator object
  flag = flag && dyn_scl.init(rds);         //Set up kinematics and dynamics object
  flag = flag && rio.init(rds.name_,rds.dof_);

  std::string Robot_name(rds.name_);

  for(unsigned int i=0;i<rds.dof_;i++){ rio.sensors_.q_(i) = rds.rb_tree_.at(i)->joint_default_pos_; }

  if(false == flag){ return 1; }            //Error check.

  /******************************ChaiGlut Graphics************************************/
  glutInit(&argc, argv); // We will use glut for the window pane (not the graphics).
  if(mode == 'k')
    flag = p.readGraphicsFromFile("../../../specs/Kuka/KukaCfg.xml","KukaBotStdView",rgr);
  if(mode == 'p')
    flag = p.readGraphicsFromFile("../../../specs/Puma/PumaCfg.xml","PumaBotStdView",rgr);
  flag = flag && rchai.initGraphics(&rgr);
  flag = flag && rchai.addRobotToRender(&rds,&rio);
  flag = flag && scl_chai_glut_interface::initializeGlutForChai(&rgr, &rchai);
  if(false==flag) { std::cout<<"\nCouldn't initialize chai graphics\n"; return 1; }

  /******************************Simulation************************************/
  // Now let us integrate the model for a variety of timesteps and see energy stability
  std::cout<<"\nIntegrating the Robot's physics. \nWill test two different controllers.\n Press (x) to exit at anytime.\n";

  //Setting simulation time.
  long iter = 0, n_iters=200000; double dt=0.0001;

  //# of threads.
  omp_set_num_threads(3);

  int thread_id; double tstart, tcurr; flag = false;


  Eigen::Vector3d hpos; //control position of off-set point wrt. hand
  if(Robot_name == "KukaBot") hpos << 0,0,0.24;
  else if(Robot_name == "PumaBot") hpos << -0.24,0,0;

  Eigen::MatrixXd Jx;
  double zi = 0.8;
  double ti = 0;
  double delta_T = 2; //2sec

  double param, theta;
  const Eigen::Vector3d offset(0,-0.5,zi+0.1); //control position of off-set of Kinect
  Eigen::Vector3d x_init, X_ball, V_ball, a_ball, X_ball_init, V_ball_init, V_ball_final;

  //Difference between two coordinate, which is used in Orientation control
  Eigen::Vector3d delta_PHI;

  //Position Control
  Eigen::Vector3d x, x_des;

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


  I.setZero(rio.dof_,rio.dof_);
  for(int i=0; i<rio.dof_; i++){
    for(int j=0; j<rio.dof_; j++) if(i==j) I(i,j) = 1;
  }

  std::cout << "dof : \n" << rio.dof_;

  //Used to rotate desired y axis to define x axis.
  Eigen::MatrixXd Rotate_90_degree; Rotate_90_degree.setZero(3,3);
  Rotate_90_degree(0,1) = 1; Rotate_90_degree(2,0) = -1; Rotate_90_degree(2,2) = 1;

  scl::SRigidBodyDyn *rhand = NULL;

  if(Robot_name == "KukaBot"){
  /*************Print Transformation matrix of each link of KUKA*************/
  std::string link_name1("end-effector");
  rhand = rgcm.rbdyn_tree_.at(link_name1);
  if(rhand == NULL){
      std::cout<<"\nERROR : Could not find the link : "<<link_name1<<" in the rbdyn tree.\n"; return 1;
  }
/*  scl::SRigidBodyDyn *link6 = rgcm.rbdyn_tree_.at("link_6");
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

  // Compute kinematic quantities
  dyn_scl.computeTransformsForAllLinks(rgcm.rbdyn_tree_,rio.sensors_.q_);
  dyn_scl.computeJacobianWithTransforms(Jx,*rhand,rio.sensors_.q_,hpos);
  if(false == flag) flag = true;
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
  }

  if(Robot_name == "PumaBot"){
    std::string link_name1("end-effector");
    rhand = rgcm.rbdyn_tree_.at(link_name1);
  }
  
  // Computer Vision Tracking with Kinect
  BallTracker BT("Green");
  //x_des << 0,0,zi-0.1;

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

      /****************************Set Initial Position for the ball*****************************/
      //*
      Vec3f position = BT.getPosition(); // x, z, y
      Vec3f velocity = BT.getVelocity();
      X_ball(0) = offset(0) + position[0]; X_ball(1) = offset(1) + position[2]; X_ball(2) = offset(2) + position[1];
      //X_ball(0) = position[0]; X_ball(1) = position[2]; X_ball(2) = position[1];
      V_ball(0) = velocity[0]; V_ball(1) = velocity[2]; V_ball(2) = velocity[1];
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

if(Robot_name == "KukaBot"){
      while((true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running) && (iter<=n_iters))
      {
        tcurr = sutil::CSystemClock::getSimTime();
        std::cout << x_des(0)<<","<<x_des(1)<<","<<x_des(2)<< std::endl;
        //*
        position = BT.getPosition(); // x, z, y
        velocity = BT.getVelocity();
        if ( position != Vec3f(0,0,0) ) { // Update if valid
        X_ball(0) = offset(0) + position[0]; X_ball(1) = offset(1) + position[2]; X_ball(2) = offset(2) + position[1];
        //X_ball(0) = position[0]; X_ball(1) = position[2]; X_ball(2) = position[1];
        V_ball(0) = velocity[0]; V_ball(1) = velocity[2]; V_ball(2) = velocity[1];
        }
        /*/
        //Motion of ball
        X_ball(0) = X_ball(0) + V_ball(0)*dt;
        X_ball(1) = X_ball(1) + V_ball(1)*dt;
        X_ball(2) = X_ball(2) + V_ball(2)*dt;
        V_ball(2) = V_ball(2) - 9.8*dt; 
        //*/
        
        std::cout << X_ball(0)<<","<<X_ball(1)<<","<<X_ball(2)<< std::endl;
                std::cout << V_ball(0)<<","<<V_ball(1)<<","<<V_ball(2)<< std::endl;
        ping_pong->setLocalPos(X_ball(0), X_ball(1), X_ball(2));
  		X_ball_init = X_ball;
        V_ball_init = V_ball;

        ti = (V_ball_init(2) + sqrt(V_ball_init(2)*V_ball_init(2) + 2*9.8*(X_ball_init(2)-zi)))/9.8;

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


        if(0.1 > abs(ti)){
            if((X_ball(2) < (zi+0.2)) && (V_ball(2) < 0)){
              x_des << X_ball(0)+V_ball(0)*dt, X_ball(1)+V_ball(1)*dt, zi+0.3;
              R1_des << 0,0,-1;

              /*
              if((X_ball(2) < zi) && (V_ball(2) < 0)) {
              V_ball(2) = -V_ball(2) + ((rand()%100)-50)/1000.0; 
              V_ball(0) = ((rand()%100)-50)/1000.0; V_ball(1) = ((rand()%100)-50)/1000.0;
              //V_ball(0) = 0; V_ball(1) = 0;
              }
              //*/
            }
            else if((X_ball(2) > zi) && (V_ball(2) >= 0)){
              x_des << X_ball(0)+V_ball(0)*dt, X_ball(1)+V_ball(1)*dt, zi-0.1;
              R1_des << 0,0,-1;
            }
        }
        else if(tcurr < ti){
          //Move the end effector to be under the predict (x,y) position of ball.
            x_des << X_ball_init(0)+V_ball_init(0)*ti, X_ball_init(1)+V_ball_init(1)*ti, zi-0.1;

            V_ball_final(2) = V_ball_init(2) -9.8*ti;


             /********Set desired rotation matrix******/
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
             R1_des = -Cross_Product(R2_des, R3_des);
              //TODO
              //R1_des << 0,0,-1;
        } else {
          std::cout<<"Time of impact is negative"<<endl;
        }

        /*************************Position Control***********************/
        Lamda_v = Jx.block(0,0,3,rio.dof_)*rgcm.M_gc_.inverse()*Jx.block(0,0,3,rio.dof_).transpose();

        Lamda_v = Lamda_v.inverse();

        F_posi = (1600 * (x_des - x) - 80*dx_v);

        F_posi_decoupled = Lamda_v * F_posi;

        /************************Orientation Control********************/
        Lamda_w = Jx.block(3,0,3,rio.dof_)*rgcm.M_gc_.inverse()*Jx.block(3,0,3,rio.dof_).transpose();

        Lamda_w = Lamda_w.inverse();

        //Get delta_PHI
        delta_PHI = -0.5*(Cross_Product(R1_curr, R1_des) + Cross_Product(R2_curr, R2_curr) + Cross_Product(R3_curr, R3_curr));

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
}
if(Robot_name == "PumaBot"){
  while((true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running) && (iter<=n_iters))
        {



          tcurr = sutil::CSystemClock::getSimTime();
          // Compute kinematic quantities
          dyn_scl.computeTransformsForAllLinks(rgcm.rbdyn_tree_,rio.sensors_.q_);
          dyn_scl.computeJacobianWithTransforms(Jx,*rhand,rio.sensors_.q_,hpos);
          if(false == flag) { x_init = rhand->T_o_lnk_ * hpos; flag = true; }

          //Get current X_global.
          x = rhand->T_o_lnk_ * hpos;

          ping_pong->setLocalPos(x_init[0], x_init[1], x_init[2]);

          //Angular Velocity
          dx_v = Jx.block(0,0,3,rio.dof_) * rio.sensors_.dq_;

          dx_w = Jx.block(3,0,3,rio.dof_) * rio.sensors_.dq_;

          /********Get current rotation matrix*******/
          R1_curr << rhand->T_o_lnk_(0,0),rhand->T_o_lnk_(1,0),rhand->T_o_lnk_(2,0);
          R2_curr << rhand->T_o_lnk_(0,1),rhand->T_o_lnk_(1,1),rhand->T_o_lnk_(2,1);
          R3_curr << rhand->T_o_lnk_(0,2),rhand->T_o_lnk_(1,2),rhand->T_o_lnk_(2,2);

          x_des = x_init;
          //R1_des << 0, 0, 1;
          R2_des << 0, 0, 1;
          //R3_des << 1, 0, 0;

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

         /******************Calculation of Joint Torque*******************/
                  rio.actuators_.force_gc_commanded_ =
                      Jx.block(0,0,3,rio.dof_).transpose() * F_posi_decoupled - 20*rio.sensors_.dq_;


         // Integrate the dynamics
         dyn_tao.integrate(rio,dt);

                   const timespec ts = {0, dt*1e9}; //0.05ms
                   nanosleep(&ts,NULL);

                    sutil::CSystemClock::tick(dt);
                    iter = iter + 1000;

        }
  sleep(5);

  tstart = sutil::CSystemClock::getSimTime();
        iter = 0;

  while((true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running) && (iter<=n_iters))
        {


          tcurr = sutil::CSystemClock::getSimTime();
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
  x_des = x_init;
                   R1_des << -1, 0, 0;
                   R2_des << 0, 0, 1;
                   R3_des << 0, 1, 0;

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
