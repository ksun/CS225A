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

double Dot_Product(Eigen::Vector3d V1, Eigen::Vector3d V2){
  double V3 = 0;
    V3 = V1(0)*V2(0) + V1(1)*V2(1) + V1(2)*V2(2);
  return V3;
}

Eigen::Vector3d Get_Reflection(Eigen::Vector3d V, Eigen::Vector3d Normal){
  Eigen::Vector3d Result;
  std::cout << "Normal : \n" << Normal.transpose();
  std::cout << "V : \n" << V.transpose();
  double num = Dot_Product(V,Normal);
  std::cout << "num: \n" << num;
  Result = V + 2*(num)*Normal;
  std::cout << "Result : \n" << Result.transpose();
  return Result;
}

Eigen::Vector3d Get_Rotation_ballVelocity(Eigen::Vector3d V_ball, Eigen::Vector3d Axis){
  Eigen::Vector3d Center = Cross_Product(Axis,V_ball);
  Center = Center / Center.norm();
  std::cout << "Center.norm() \n" << Center.norm() << std::endl;
  double Dot_ = Dot_Product(Axis, V_ball);
  double length1 = V_ball.norm(); double length2 = Axis.norm();
  double x = length1 * length2;
  x = Dot_ * (1 / x);
  double angle = acos(x);
  angle = angle * (180/PI);
  angle = angle * (2);
  std::cout <<  "angle: \n" << angle << std::endl;
  //angle = -angle;

  double eular1, eular2, eular3, eular4;
  eular1 = Center(0) * sin(angle * (0.5));
  eular2 = Center(1) * sin(angle * (0.5));
  eular3 = Center(2) * sin(angle * (0.5));
  eular4 = cos(angle * (0.5));
  Eigen::MatrixXd Rotation_M; Rotation_M.setZero(3,3);

  Rotation_M(0,0) = 1 - 2*eular2*eular2 - 2*eular3*eular3;
  Rotation_M(0,1) = 2*(eular1*eular2 + eular3*eular4);
  Rotation_M(0,2) = 2*(eular1*eular3 - eular2*eular4);

  Rotation_M(1,0) = 2*(eular1*eular2 - eular3*eular4);
  Rotation_M(1,1) = 1 - 2*eular1*eular1 - 2*eular3*eular3;
  Rotation_M(1,2) = 2*(eular2*eular3 + eular1*eular4);

  Rotation_M(2,0) = 2*(eular1*eular3 + eular2*eular4);
  Rotation_M(2,1) = 2*(eular2*eular3 - eular1*eular4);
  Rotation_M(2,2) = 1 - 2*eular1*eular1 - 2*eular2*eular2;

  Eigen::Vector3d Result = Rotation_M * V_ball;
  Result = Result / Result.norm();
  std::cout << "Result : " << Result.transpose() << std::endl;
  return Result;
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
    flag = p.readRobotFromFile("../../specs/Kuka/KukaCfg.xml","../../specs/","KukaBot",rds);
  if(mode == 'p')
    flag = p.readRobotFromFile("../../specs/Puma/PumaCfg.xml","../../specs/","PumaBot",rds);
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
    flag = p.readGraphicsFromFile("../../specs/Kuka/KukaCfg.xml","KukaBotStdView",rgr);
  if(mode == 'p')
    flag = p.readGraphicsFromFile("../../specs/Puma/PumaCfg.xml","PumaBotStdView",rgr);
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
  omp_set_num_threads(2);

  int thread_id; double tstart, tcurr; flag = false;


  Eigen::Vector3d hpos; //control position of off-set point wrt. hand
  if(Robot_name == "KukaBot") hpos << 0,0,0.24;
  else if(Robot_name == "PumaBot") hpos << -0.24,0,0;

  Eigen::MatrixXd Jx;

  Eigen::Vector3d x_init;
  Eigen::Vector3d high_point;
  high_point << 0,0,10;

  /*********Position Control and Orientation Control**********************/

  //Difference between two coordinate, which is used in Orientation control
  Eigen::Vector3d delta_PHI;

  //Position Control
  Eigen::Vector3d x;
  Eigen::Vector3d x_des (0,0,0);

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
  //Eigen::MatrixXd Rotate_90_degree; Rotate_90_degree.setZero(3,3);
  //Rotate_90_degree(0,1) = 1; Rotate_90_degree(2,0) = -1; Rotate_90_degree(2,2) = 1;

  scl::SRigidBodyDyn *rhand = NULL;
  if(Robot_name == "KukaBot"){
    /*************Print Transformation matrix of each link of KUKA*************/
    std::string link_name1("end-effector");
    rhand = rgcm.rbdyn_tree_.at(link_name1);
    if(rhand == NULL){
      std::cout<<"\nERROR : Could not find the link : "<<link_name1<<" in the rbdyn tree.\n"; return 1;
    }

  }
  if(Robot_name == "PumaBot"){
    std::string link_name1("end-effector");
    rhand = rgcm.rbdyn_tree_.at(link_name1);
  }


#pragma omp parallel private(thread_id)
  {
    thread_id = omp_get_thread_num();
    if(thread_id==1) //Simulate physics and update the rio data structure..
    {
      /******************************Draw a ping pong ball************************************/
      chai3d::cShapeSphere * ping_pong = new chai3d::cShapeSphere(0.03); // create sphere
      rchai.getChaiData()->chai_world_->addChild(ping_pong); // insert object into world

      chai3d::cShapeSphere * highest_point = new chai3d::cShapeSphere(0.05); // create sphere
      rchai.getChaiData()->chai_world_->addChild(highest_point); // insert object into world
      highest_point->setLocalPos(0, 0, 10);

      // Controller 2 : Operational space controller
      std::cout<<"\n\n***************************************************************"
          <<"\n Starting op space (task coordinate) controller..."
          <<"\n This will make a trajectory of ping pong ball"
          <<"\n***************************************************************";
      tstart = sutil::CSystemClock::getSimTime();
      iter = 0;

      char key;
      std::cin >> key;

      if(Robot_name == "KukaBot"){
              if(key == 'd'){
                while(true){
                  char dir;
                  std::cout << "Enter a direction : \n";
                  std::cin >> dir;
                  iter = 0;
                  if(dir == 'u') {
                    std::cout << "up \n";
                    R1_des << 0,0,1;
                  }
                  else if(dir == 'd'){
                    std::cout << "down\n";
                    //x_des << 0.6,-0.6,0.4;
                    R1_des << 0,0,-1;

                  }
                  else if(dir == 'r'){
                    std::cout << "right \n";
                    R1_des << 0,1,0;
                  }
                  else if(dir == 'l') {
                    std::cout << "left \n";
                    R1_des << 0,-1,0;
                  }
                  else if(dir == 'q') break;

                  ping_pong->setLocalPos(0.6, 0.6, 0.4);

                  while((true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running) && (iter<=n_iters))
                  {

                    tcurr = sutil::CSystemClock::getSimTime();

                    x_des << 0.6,0.6,0.4;

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

                    /*************************Position Control***********************/
                    Lamda_v = Jx.block(0,0,3,rio.dof_)*rgcm.M_gc_.inverse()*Jx.block(0,0,3,rio.dof_).transpose();

                    Lamda_v = Lamda_v.inverse();

                    F_posi = (1600 * (x_des - x) - 80*dx_v);

                    F_posi_decoupled = Lamda_v * F_posi;
                    /************************Orientation Control********************/
                    Lamda_w = Jx.block(3,0,3,rio.dof_)*rgcm.M_gc_.inverse()*Jx.block(3,0,3,rio.dof_).transpose();

                    Lamda_w = Lamda_w.inverse();

                    //Get delta_PHI
                    delta_PHI = -0.5*(Cross_Product(R1_des, R1_curr) + Cross_Product(R2_curr, R2_curr) + Cross_Product(R3_curr, R3_curr));

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
                    iter = iter + 5;
                  }
                }
              }

              if(key == 'c'){
                while(true){
                  std::cout << "Start! or Quit (q) : \n" << std::endl;
                  char temp;
                  std::cin >> temp;
                  if(temp == 'q') break;

                  tstart = sutil::CSystemClock::getSimTime();
                  iter = 0;

                  Eigen::Vector3d ball_force, X_ball, V_ball, a_ball, V_ball_store, ez;
                  Eigen::Vector3d ball_force_est, X_ball_est, V_ball_est, a_ball_est;
                  Eigen::Vector3d gravity (0,0,-9.81);

                  bool hitting = false, impact = false;
                  float ball_pos_threshold = 0.6, radius = 0.025, rho = 1, Cd = .25, mass = .002, impact_time = (10000*dt), Delta_T = 0.0001;
                  int est_iter = 0, iters_impact = 0;
                  Eigen::Vector3d X_ball_init (-0.65,-0.65,4);
                  Eigen::Vector3d V_ball_init (0,0,0);
                  ez << 0,0,1;
                  X_ball = X_ball_init;
                  V_ball = V_ball_init;

                  while((true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running) && (iter<=n_iters))
                  {
                    tcurr = sutil::CSystemClock::getSimTime();

                    ping_pong->setLocalPos(X_ball(0), X_ball(1), X_ball(2));

                    /********Get current rotation matrix*******/
                    R1_curr << rhand->T_o_lnk_(0,0),rhand->T_o_lnk_(1,0),rhand->T_o_lnk_(2,0);
                    R2_curr << rhand->T_o_lnk_(0,1),rhand->T_o_lnk_(1,1),rhand->T_o_lnk_(2,1);
                    R3_curr << rhand->T_o_lnk_(0,2),rhand->T_o_lnk_(1,2),rhand->T_o_lnk_(2,2);

                    /*******Control Logic************/

                  if(iter >= iters_impact){
                  if(X_ball(2)< x(2)+0.025){ //checks for impact
                      //std::cout<<"\n z pos end effector"<<x(2);
                      Eigen::MatrixXd delme = V_ball.transpose() * R1_curr;
                      double xx = 2*delme(0,0);
                      xx = xx/R1_curr.norm();
                      V_ball -= xx * R1_curr.normalized();
                      V_ball = 1.05*V_ball.normalized();
                      V_ball_store = V_ball;

                      //std::cout<<"\n ball vel"<<V_ball.transpose();

                      int xxx;
                      x_des(2)=ball_pos_threshold-.1;
                      impact=true;
                      hitting = false;
                      iters_impact=iter+7000;
                      impact_time=(iter+1000000)*dt;
                  }else{ //if no impact
                      if(hitting){       //check to see if hitting
                        x_des(2)=ball_pos_threshold+.5;
                      }else if(iter*dt>impact_time-Delta_T){ //check to see it should be hitting
                        hitting=true;
                        //std::cout<<"\n hitting";
                      }else{ //if no impact and not hitting esitmate trejectory
                        X_ball_est = X_ball; //intialize for estimation
                        V_ball_est = V_ball;
                        est_iter = 0;
                        //std::cout<<"\n**********************EST*********************";
                        while(X_ball_est(2) > ball_pos_threshold){//estimated trajectory
                          double xx = .5*rho*3.14159*radius*radius*Cd*
                          V_ball_est.norm();
                          ball_force_est = -1* xx*V_ball_est;
                          a_ball_est=gravity+(ball_force_est/mass);
                          V_ball_est=V_ball_est+a_ball_est*dt;
                          X_ball_est=X_ball_est+V_ball_est*dt;
                          est_iter++;
                        }
                        std::cout << "x est: " << X_ball_est.transpose() << std::endl;
                        impact_time=(iter+est_iter)*dt;
                        if(X_ball_est.norm() < 1.25){
                          x_des << X_ball_est(0), X_ball_est(1), ball_pos_threshold-.1;
                          R1_des=(ez-V_ball_est)/sqrt(2*V_ball_est.norm()-V_ball_est.transpose()*ez);
                          R1_des=R1_des.normalized()*.2;
                          R1_des = R1_des/R1_des.norm();
                        }
                        else{
                          x_des << 0.4,0.4,0.4;
                          R1_des << 0,0,1;
                        }
                    }
            //computing ball forces since no impact
                      double xxxx=.5*rho*3.14159*radius*radius*Cd*V_ball.norm();
                      //std::cout<<"xxxx is "<<xxxx;
                      ball_force=-1*xxxx*V_ball;
                      a_ball=gravity+ball_force/mass;
                      V_ball=V_ball+a_ball*dt;
                  }
                    }else{
                      x_des(2)=ball_pos_threshold-.1;
                      V_ball=V_ball_store;
                      //std::cout<<"\n x des "<<x_des.transpose();
                    }


                    X_ball = X_ball + V_ball*dt;

                    //std::cout << " Position of the ball :" << X_ball.transpose() << std::endl;
                    // Compute kinematic quantities
                    dyn_scl.computeTransformsForAllLinks(rgcm.rbdyn_tree_,rio.sensors_.q_);
                    dyn_scl.computeJacobianWithTransforms(Jx,*rhand,rio.sensors_.q_,hpos);
                    if(false == flag) { x_init = rhand->T_o_lnk_ * hpos; flag = true; }

                    //Get current X_global.
                    x = rhand->T_o_lnk_ * hpos;

                    //Angular Velocity
                    dx_v = Jx.block(0,0,3,rio.dof_) * rio.sensors_.dq_;

                    dx_w = Jx.block(3,0,3,rio.dof_) * rio.sensors_.dq_;

                    /*****calculation of torque***********/

                    /*************************Position Control***********************/
                    Lamda_v = Jx.block(0,0,3,rio.dof_)*rgcm.M_gc_.inverse()*Jx.block(0,0,3,rio.dof_).transpose();

                    Lamda_v = Lamda_v.inverse();

                    F_posi = (1600 * (x_des - x) - 80*dx_v);

                    F_posi_decoupled = Lamda_v * F_posi;
                    /************************Orientation Control********************/
                    Lamda_w = Jx.block(3,0,3,rio.dof_)*rgcm.M_gc_.inverse()*Jx.block(3,0,3,rio.dof_).transpose();

                    Lamda_w = Lamda_w.inverse();

                    //Get delta_PHI
                    delta_PHI = -0.5*(Cross_Product(R1_des, R1_curr) + Cross_Product(R2_curr, R2_curr) + Cross_Product(R3_curr, R3_curr));

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
              }
            }

      if(Robot_name == "PumaBot"){
        if(key == 'd'){
          while(true){
            char dir;
            std::cout << "Enter a direction : \n";
            std::cin >> dir;
            iter = 0;
            if(dir == 'u') {
              std::cout << "up \n";
              R2_des << 0,0,1;
            }
            else if(dir == 'd'){
              std::cout << "down\n";
              //x_des << 0.6,-0.6,0.4;
              R2_des << 0,0,-1;

            }
            else if(dir == 'r'){
              std::cout << "right \n";
              R2_des << 0,1,0;
            }
            else if(dir == 'l') {
              std::cout << "left \n";
              R2_des << 0,-1,0;
            }
            else if(dir == 'q') break;

            ping_pong->setLocalPos(0.6, 0.6, 0.4);

            while((true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running) && (iter<=n_iters))
            {

              tcurr = sutil::CSystemClock::getSimTime();

              x_des << 0.6,0.6,0.4;

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
              iter = iter + 5;


            }

          }
        }

        if(key == 'c'){
          while(true){
            std::cout << "Start! or Quit (q) : \n" << std::endl;
            char temp;
            std::cin >> temp;
            if(temp == 'q') break;

            tstart = sutil::CSystemClock::getSimTime();
            iter = 0;

            Eigen::Vector3d ball_force, X_ball, V_ball, a_ball, V_ball_store, ez;
            Eigen::Vector3d ball_force_est, X_ball_est, V_ball_est, a_ball_est;
            Eigen::Vector3d gravity (0,0,-9.81);
            Eigen::Vector3d x_des_prev, R2_des_prev;
            bool hitting = false, impact = false;
            float ball_pos_threshold = 0.6, radius = 0.025, rho = 1, Cd = .25, mass = .002, impact_time = (10000*dt), Delta_T = 0.0001;
            int est_iter = 0, iters_impact = 0;
            Eigen::Vector3d X_ball_init (0.6,0.6,4);
            Eigen::Vector3d V_ball_init (0,0,0);
            ez << 0,0,1;
            X_ball = X_ball_init;
            V_ball = V_ball_init;


            while((true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running) && (iter<=n_iters))
            {
              tcurr = sutil::CSystemClock::getSimTime();

              ping_pong->setLocalPos(X_ball(0), X_ball(1), X_ball(2));

              /********Get current rotation matrix*******/
              R1_curr << rhand->T_o_lnk_(0,0),rhand->T_o_lnk_(1,0),rhand->T_o_lnk_(2,0);
              R2_curr << rhand->T_o_lnk_(0,1),rhand->T_o_lnk_(1,1),rhand->T_o_lnk_(2,1);
              R3_curr << rhand->T_o_lnk_(0,2),rhand->T_o_lnk_(1,2),rhand->T_o_lnk_(2,2);

              /*******Control Logic************/

          //if((X_ball_est(0)*X_ball_est(0) + X_ball_est(1)*X_ball_est(1)) < 0.36){
            if(iter >= iters_impact){
            if(X_ball(2)< x(2)+0.025){ //checks for impact
                std::cout<<"\n z pos end effector"<<x(2);
                Eigen::MatrixXd delme = V_ball.transpose() * R2_curr;
                double xx = 2*delme(0,0);
                xx = xx/R2_curr.norm();
                V_ball -= xx * R2_curr.normalized();
                V_ball = 1.05*V_ball.normalized();
                V_ball_store = V_ball;

                std::cout<<"\n ball vel"<<V_ball.transpose();

                int xxx;
                x_des(2)=ball_pos_threshold-.1;
                impact=true;
                hitting = false;
                iters_impact=iter+7000;
                impact_time=(iter+1000000)*dt;
            }else{ //if no impact
                if(hitting){       //check to see if hitting
                  x_des(2)=ball_pos_threshold+.5;
                }else if(iter*dt>impact_time-Delta_T){ //check to see it should be hitting
                  hitting=true;
                  std::cout<<"\n hitting";
                }else{ //if no impact and not hitting esitmate trejectory
                  X_ball_est = X_ball; //intialize for estimation
                  V_ball_est = V_ball;
                  est_iter = 0;
                  //std::cout<<"\n**********************EST*********************";
                  while(X_ball_est(2) > ball_pos_threshold){//estimated trajectory
                    double xx = .5*rho*3.14159*radius*radius*Cd*
                    V_ball_est.norm();
                    ball_force_est = -1* xx*V_ball_est;
                    a_ball_est=gravity+(ball_force_est/mass);
                    V_ball_est=V_ball_est+a_ball_est*dt;
                    X_ball_est=X_ball_est+V_ball_est*dt;
                    est_iter++;
                  }
                  impact_time=(iter+est_iter)*dt;
                  if(X_ball_est.norm() < 1.5){
                    x_des << X_ball_est(0), X_ball_est(1), ball_pos_threshold-.1;
                    R2_des=(ez-V_ball_est)/sqrt(2*V_ball_est.norm()-V_ball_est.transpose()*ez);
                    R2_des=R2_des.normalized()*.2;
                    R2_des = R2_des/R2_des.norm();
                  }
                  else{
                    x_des << 0.4,0.4,0.4;
                    R2_des << 0,0,1;
                  }
              }
      //computing ball forces since no impact
                double xxxx=.5*rho*3.14159*radius*radius*Cd*V_ball.norm();
                std::cout<<"xxxx is "<<xxxx;
                ball_force=-1*xxxx*V_ball;
                a_ball=gravity+ball_force/mass;
                V_ball=V_ball+a_ball*dt;
            }
              }else{
                x_des(2)=ball_pos_threshold-.1;
                V_ball=V_ball_store;
                std::cout<<"\n x des "<<x_des.transpose();
              }


              X_ball = X_ball + V_ball*dt;
              std::cout << " Position of the ball :" << X_ball.transpose() << std::endl;
              // Compute kinematic quantities
              dyn_scl.computeTransformsForAllLinks(rgcm.rbdyn_tree_,rio.sensors_.q_);
              dyn_scl.computeJacobianWithTransforms(Jx,*rhand,rio.sensors_.q_,hpos);
              if(false == flag) { x_init = rhand->T_o_lnk_ * hpos; flag = true; }

              //Get current X_global.
              x = rhand->T_o_lnk_ * hpos;

              //Angular Velocity
              dx_v = Jx.block(0,0,3,rio.dof_) * rio.sensors_.dq_;

              dx_w = Jx.block(3,0,3,rio.dof_) * rio.sensors_.dq_;

              /*****calculation of torque***********/

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
        }
      }
      //Then terminate
      scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running = false;
    }
    else{  //Read the rio data structure and updated rendererd robot..
      while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      {
        glutMainLoopEvent();
        const timespec ts = {0, 15000000};/*15ms*/
        nanosleep(&ts,NULL);
      }
    }
  }

  /******************************Exit Gracefully************************************/
  std::cout<<"\n\nExecuted Successfully";
  std::cout<<"\n**********************************\n"<<std::flush;

  return 0;
}
