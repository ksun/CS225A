// controlDLL.cpp : Defines the entry point for the DLL application.
//
#pragma warning( disable : 4335)

#ifdef WIN32
#include "stdafx.h"
BOOL APIENTRY DllMain( HANDLE hModule, 
                       DWORD  ul_reason_for_call, 
                       LPVOID lpReserved
					 )
{
    return TRUE;
}

#include <stdio.h>

#else //#ifdef WIN32
#include "servo.h"

#endif //#ifdef WIN32

#include "param.h"
#include "control.h"
//#include "UiAgent.h"
#include "PrVector.h"
#include "PrMatrix.h"
#include "Utils.h" // misc. utility functions, such as toRad, toDeg, etc.
#include <math.h>
#include <algorithm>
using std::min;
using std::max;


void FindCubicSpline(GlobalVariables& gv);
void FindOperationalCubicSpline(GlobalVariables& gv);
void EvaluateCubicSpline( PrVector& pos, PrVector& vel, PrVector& acc, GlobalVariables& gv );
bool inv_kin( const PrVector3& pos, const PrMatrix3& rot, int elbow,
              PrVector& qOut, GlobalVariables& gv );
void getSaturatedFprime( PrVector& fPrime, GlobalVariables& gv );
void OpDynamics( const PrVector& fPrime, GlobalVariables& gv );

static PrVector splineParam0;// params used for track splines
static PrVector splineParam1F;
static PrVector splineParam0F;
static PrVector splineParam2;
static PrVector splineParam2F;
static PrVector splineParam3;
static PrVector6 tf, tf2;
static Float splineStartTime, splineDuration, splineDuration2, finishTime1, initTime;
double storageQmax3, storageQmax5;
#define PI 3.1415926535897;

// *******************************************************************
// Initialization functions
// *******************************************************************

void InitControl(GlobalVariables& gv) 
{
   // This code runs before the first servo loop
}

void PreprocessControl(GlobalVariables& gv)
{
   // This code runs on every servo loop, just before the control law
}


void PostprocessControl(GlobalVariables& gv) 
{
   // This code runs on every servo loop, just after the control law
   Float MIN_RHO = 1e-5;
   if( gv.jlimit )
   {
      for( int ii = 0; ii < gv.dof; ii++ )
      {
         if( gv.q[ii] < gv.qmin[ii] + gv.q0[ii] )
         {
            Float rho = max( gv.q[ii] - gv.qmin[ii], MIN_RHO );
            Float rhoi = gv.q0[ii];
            gv.tau[ii] += gv.kj[ii] * ( 1/rho - 1/rhoi ) / ( rho * rho );
         }
         else if( gv.q[ii] > gv.qmax[ii] - gv.q0[ii] )
         {
            Float rho = max( gv.qmax[ii] - gv.q[ii], MIN_RHO );
            Float rhoi = gv.q0[ii];
            gv.tau[ii] -= gv.kj[ii] * ( 1/rho - 1/rhoi ) / ( rho * rho );
         }
      }
   }
}

void initFloatControl(GlobalVariables& gv) 
{
}

void initOpenControl(GlobalVariables& gv) 
{
}

void initNjholdControl(GlobalVariables& gv) 
{
   gv.qd = gv.q;
   gv.dqd.zero();
}

void initJholdControl(GlobalVariables& gv) 
{
   initNjholdControl(gv);
}

void initNjmoveControl(GlobalVariables& gv) 
{
}

void initJmoveControl(GlobalVariables& gv) 
{
}

void initNjgotoControl(GlobalVariables& gv) 
{
   for( int ii = 0; ii < gv.dof; ii++ )
   {
      gv.qd[ii] = min( max( gv.qd[ii], gv.qmin[ii] ), gv.qmax[ii] );
   }
} 

void initJgotoControl(GlobalVariables& gv) 
{
   initNjgotoControl(gv);
}

void initNjtrackControl(GlobalVariables& gv) 
{
   FindCubicSpline(gv);
}

void initJtrackControl(GlobalVariables& gv) 
{
   FindCubicSpline(gv);
}

void initNxtrackControl(GlobalVariables& gv) 
{
   PrVector6 local_qd;
   bool success = inv_kin( gv.Td.translation(),
                           gv.Td.rotation().matrix(), gv.elbow, local_qd, gv );
   if( success )
   {
      switch( gv.dof )
      {
      case 1:
         gv.qd[0] = local_qd[1];
         break;
      case 3:
         gv.qd[0] = local_qd[1];
         gv.qd[1] = local_qd[2];
         gv.qd[2] = local_qd[4];
         break;
      case 4:
         gv.qd[0] = local_qd[0];
         gv.qd[1] = local_qd[1];
         gv.qd[2] = local_qd[2];
         gv.qd[3] = local_qd[4];
         break;
      case 6:
         gv.qd = local_qd;
         break;
      }
   }
   else
   {
      gv.qd = gv.q;
   }
   gv.dqd.zero();
   FindCubicSpline(gv);
}

void initXtrackControl(GlobalVariables& gv) 
{
   initNxtrackControl(gv);
} 

void initNholdControl(GlobalVariables& gv) 
{
   gv.xd = gv.x;
   gv.dxd.zero();
}

void initHoldControl(GlobalVariables& gv) 
{
   initNholdControl(gv);
}

void initNgotoControl(GlobalVariables& gv) 
{
} 

void initGotoControl(GlobalVariables& gv) 
{
} 

void initNtrackControl(GlobalVariables& gv) 
{
   FindOperationalCubicSpline(gv);
}

void initTrackControl(GlobalVariables& gv) 
{
   FindOperationalCubicSpline(gv);
} 

void initPfmoveControl(GlobalVariables& gv) 
{
} 

void initLineControl(GlobalVariables& gv) 
{
}

void initProj1Control(GlobalVariables& gv) 
{
	PrVector6 qThrow, dqThrow, qFinish;
	bool safetyJoint1, safetyJoint35;
	
// Initialization of q, dq and ddqd
	gv.qd.zero();
	gv.dqd.zero();
	gv.ddqd.zero();

	static double tableHeight = 1.2; // Height of the coord0 
	static double robotHeight = 1.2791;//1.3841; // End-effector height from the {0} frame in the specific config
	static double robotXin1  = 0.2910; // End-effector x position in frame1
	static double Lplus = 0.5; // Length of the original effector

	double q1Low	=	-20.0/180.0*PI; // Minimum angle for joint1
	double q1Upper	=	45.0/180.0*PI; // Maximum angle for joint1
	
	double theta	= 0.0/180.0*PI;//0.0;	// Orientation from the base (Got from the vision)
	double baseR	= 3.80;	// Horizontal distance from the base (Got from the vision)
	double r = baseR*sqrt(1-pow(L1/baseR,2)) - robotXin1; // Distance required for ball to fly
	double vx;				// Required velocity

	double objectHeight = 0.30; // Height of the target
	double alpha = 1.8; // The ratio of dq2 and dq3 (Because we have a freedom of choice)
	
	vx = sqrt(9.80665*pow(r,2)/(2*(tableHeight + robotHeight)-2*objectHeight));

	if(theta - asin(L1/baseR) > q1Low && theta - asin(L1/baseR) < q1Upper)
	{
		safetyJoint1 = true;
	}
	else
	{
		safetyJoint1 = false;
	}

	// Jacobian in this config
	dqThrow[2] = vx/(4881.0/10000.0 + Lplus + (11/200.0 + Lplus)*alpha);
	dqThrow[4] = alpha*dqThrow[2];

	if((abs(dqThrow[2]) < gv.dqmax[2]) && (abs(dqThrow[4]) < gv.dqmax[4]))
	{
		safetyJoint35 = true;
	}
	else
	{
		safetyJoint35 = false;
	}

	if(safetyJoint1 && safetyJoint35)
	{
		// Desired starting posture
		gv.qd[0] = theta - asin(L1/baseR);
		gv.qd[1] = -45.0/180.0*PI;
		gv.qd[2] = -30.0/180.0*PI;
		gv.qd[4] = -70.0/180.0*PI;

		// Release point
		qThrow[2] = 45.0/180.0*PI;
		qThrow[4] = 0.0/180.0*PI;

		splineParam0 = gv.qd;
		splineParam2.setSize( gv.dof, true );
		splineParam0F.setSize( gv.dof, true );
		splineParam1F.setSize( gv.dof, true );
		splineParam2F.setSize( gv.dof, true );

		splineParam2[2] = pow(dqThrow[2],2)/(4.0*(qThrow[2] - gv.qd[2]));
		splineParam2[4] = pow(dqThrow[4],2)/(4.0*(qThrow[4] - gv.qd[4]));

		// Time to reach the release point
		tf[2] = dqThrow[2]/(2.0*splineParam2[2]);
		tf[4] = dqThrow[4]/(2.0*splineParam2[4]);

		// Deceleration Phase
		qFinish[2] = qThrow[2]  + (qThrow[2] - gv.qd[2]);
		qFinish[4] = qThrow[4]  + (qThrow[4] - gv.qd[4]);

		splineParam0F[2] = qThrow[2];
		splineParam0F[4] = qThrow[4];
		splineParam1F[2] = dqThrow[2];
		splineParam1F[4] = dqThrow[4];
		splineParam2F[2] = -pow(dqThrow[2],2)/(4.0*(qFinish[2] - qThrow[2]));
		splineParam2F[4] = -pow(dqThrow[4],2)/(4.0*(qFinish[4] - qThrow[4]));

		gv.ddqd[2] = 2.0*splineParam2F[2];
		gv.ddqd[4] = 2.0*splineParam2F[4];

		// Time to reach the final config
		tf2[2] = -splineParam1F[2]/(2.0*splineParam2F[2]);
		tf2[4] = -splineParam1F[4]/(2.0*splineParam2F[4]);

		// Duration of acceleration and deceleration spline
		splineDuration = max(tf[2],tf[4]);
		splineDuration2 = max(tf2[2],tf2[4]);

	}
	else
	{
		splineDuration = 0.0;
		splineDuration2 = 0.0;
		gv.dqmax[2] = 80.0/180*PI;
		gv.dqmax[4] = 80.0/180*PI;
	}

	storageQmax3 = gv.dqmax[2]; 
	storageQmax5 = gv.dqmax[4];

	// Time required to move to the initial config
	initTime = 4.0;
	splineStartTime = gv.curTime + initTime;

	// Acceleration phase ending time
	finishTime1 = splineStartTime + splineDuration;
}

void initProj2Control(GlobalVariables& gv) 
{
}

void initProj3Control(GlobalVariables& gv) 
{
}


// *******************************************************************
// Control laws
// *******************************************************************

void noControl(GlobalVariables& gv)
{
}

void floatControl(GlobalVariables& gv)
{
   gv.tau = gv.G;
}

void openControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void njholdControl(GlobalVariables& gv) 
{
   gv.tau = -gv.kp * (gv.q - gv.qd) - gv.kv * (gv.dq - gv.dqd);
}

void jholdControl(GlobalVariables& gv) 
{
   jmoveControl(gv);
}

void njmoveControl(GlobalVariables& gv)
{
   gv.tau = -gv.kp * (gv.q - gv.qd) - gv.kv * (gv.dq - gv.dqd) + gv.G;
}

void jmoveControl(GlobalVariables& gv)
{
   PrVector tauPrime = -gv.kp * (gv.q - gv.qd) - gv.kv * (gv.dq - gv.dqd);
   gv.tau = gv.A * tauPrime + gv.B + gv.G;
}

void njgotoControl(GlobalVariables& gv) 
{
   static const Float MIN_KV = 1e-4;
   PrVector local_dqd = gv.dqd;

   for( int ii = 0; ii < gv.dof; ii++ )
   {
      local_dqd[ii] -= (gv.q[ii] - gv.qd[ii]) * gv.kp[ii] / max(gv.kv[ii], MIN_KV);
      local_dqd[ii] = min(max(local_dqd[ii], -gv.dqmax[ii]), gv.dqmax[ii]);
   }
   gv.tau = -gv.kv * (gv.dq - local_dqd) + gv.G;
}

void jgotoControl(GlobalVariables& gv) 
{
   static const Float MIN_KV = 1e-4;
   PrVector local_dqd = gv.dqd;

   for( int ii = 0; ii < gv.dof; ii++ )
   {
      local_dqd[ii] -= (gv.q[ii] - gv.qd[ii]) * gv.kp[ii] / max(gv.kv[ii], MIN_KV);
      local_dqd[ii] = min(max(local_dqd[ii], -gv.dqmax[ii]), gv.dqmax[ii]);
   }
   PrVector tauPrime = -gv.kv * (gv.dq - local_dqd);
   gv.tau = gv.A * tauPrime + gv.B + gv.G;
}

void njtrackControl(GlobalVariables& gv) 
{
   PrVector local_qd( gv.dof );
   PrVector local_dqd( gv.dof );
   PrVector local_ddqd( gv.dof );
   EvaluateCubicSpline( local_qd, local_dqd, local_ddqd, gv );

   gv.tau = -gv.kp * (gv.q-local_qd) - gv.kv * (gv.dq-local_dqd) + local_ddqd + gv.G;
}

void jtrackControl(GlobalVariables& gv)
{
   PrVector local_qd( gv.dof );
   PrVector local_dqd( gv.dof );
   PrVector local_ddqd( gv.dof );
   EvaluateCubicSpline( local_qd, local_dqd, local_ddqd, gv );

   PrVector tauPrime =
      -gv.kp * (gv.q-local_qd) - gv.kv * (gv.dq-local_dqd) + local_ddqd;
   gv.tau = gv.A * tauPrime + gv.B + gv.G;
}

void nxtrackControl(GlobalVariables& gv) 
{
   njtrackControl(gv);
}

void xtrackControl(GlobalVariables& gv) 
{
   jtrackControl(gv);
}

void nholdControl(GlobalVariables& gv) 
{
   ngotoControl(gv);
}

void holdControl(GlobalVariables& gv) 
{
   gotoControl(gv);
}

void ngotoControl(GlobalVariables& gv) 
{
   PrVector fPrime;
   getSaturatedFprime( fPrime, gv );
   gv.tau = gv.Jtranspose * fPrime + gv.G;
}

void gotoControl(GlobalVariables& gv) 
{
   PrVector fPrime;
   getSaturatedFprime( fPrime, gv );
   OpDynamics( fPrime, gv );
}

void ntrackControl(GlobalVariables& gv) 
{
   PrVector local_xd;
   PrVector local_dxd;
   PrVector local_ddxd;  // ignored
   EvaluateCubicSpline( local_xd, local_dxd, local_ddxd, gv );

   PrVector fPrime = ( -gv.kp * (gv.Einverse * (gv.x-local_xd))
                       - gv.kv * (gv.dx - gv.Einverse * local_dxd) );
   gv.tau = gv.Jtranspose * fPrime + gv.G;
}

void trackControl(GlobalVariables& gv) 
{
   PrVector local_xd;
   PrVector local_dxd;
   PrVector local_ddxd;  // ignored
   EvaluateCubicSpline( local_xd, local_dxd, local_ddxd, gv );

   PrVector fPrime = ( -gv.kp * (gv.Einverse * (gv.x-local_xd))
                       - gv.kv * (gv.dx - gv.Einverse * local_dxd) );
   OpDynamics( fPrime, gv );
}

void pfmoveControl(GlobalVariables& gv) 
{
   Float MIN_RHO = 1e-5;

   // Calculate the force to get us to the goal position

   PrVector fPrime;
   getSaturatedFprime( fPrime, gv );

   // Now add the repulsive force from each obstacle

   PrVector3 currentPos = gv.T.translation();
   for( int ii = 0; ii < gv.numObstacles; ii++ )
   {
      PrVector3 lineToObstacle = gv.obstacles[ii].coord - currentPos;
      Float rho = lineToObstacle.magnitude() - gv.obstacles[ii].radius;
      if( rho < gv.rho0 )
      {
         rho = max( rho, MIN_RHO );
         PrVector3 linearForce =
            -gv.eta * lineToObstacle * (1/rho - 1/gv.rho0) / (rho*rho);
         if( gv.dof == 3 )
         {
            fPrime[0] += linearForce[0];
            fPrime[1] += linearForce[2];
         }
         else if( gv.dof >= 4 )
         {
            fPrime[0] += linearForce[0];
            fPrime[1] += linearForce[1];
            fPrime[2] += linearForce[2];
         }
      }
   }

   OpDynamics( fPrime, gv );
}

void lineControl(GlobalVariables& gv)
{
   // Find the closest point on the line to the current position
   PrVector3 currentPos = gv.T.translation();
   PrVector3 closestPoint = gv.line.center() +
      gv.line.unitVec() * gv.line.unitVec().dot( currentPos - gv.line.center() );

   // Set that point to be the desired position
   PrVector deltaX = gv.x - gv.xd;
   if( gv.dof == 3 )
   {
      deltaX[0] = gv.x[0] - closestPoint[0];
      deltaX[1] = gv.x[1] - closestPoint[2];
   }
   else if( gv.dof >= 4 )
   {
      deltaX[0] = gv.x[0] - closestPoint[0];
      deltaX[1] = gv.x[1] - closestPoint[1];
      deltaX[2] = gv.x[2] - closestPoint[2];
   }

   // Find gv.tau

   PrVector fPrime = -gv.kp * ( gv.Einverse * deltaX ) - gv.kv * gv.dx;
   OpDynamics( fPrime, gv );
}

void proj1Control(GlobalVariables& gv) 
{
	double t;
	if (gv.curTime < splineStartTime )
	{
		gv.dqmax[2] = 80/180.0*PI;
		gv.dqmax[4] = 80/180.0*PI;
		jgotoControl(gv);
	}
	else
	{
		gv.dqmax[2] = storageQmax3;
		gv.dqmax[4] = storageQmax5;
		// Torque input for acceleration phase
		if(gv.curTime - splineStartTime <  splineDuration) 
		{
			if (tf[2] > finishTime1 - gv.curTime)
			{
				t = gv.curTime - (finishTime1 - tf[2]);
				gv.qd[2] =  splineParam0[2] + splineParam2[2]*pow(t,2);
				gv.dqd[2] = 2.0*splineParam2[2]*t;
				gv.ddqd[2] = 2.0*splineParam2[2];
			}
			else
			{
				gv.qd[2] = gv.qd[2];
				gv.dqd[2] = 0;
			}

			if (tf[4] > finishTime1 - gv.curTime)
			{
				t = gv.curTime - (finishTime1 - tf[4]);
				gv.qd[4] =  splineParam0[4] + splineParam2[4]*pow(t,2);
				gv.dqd[4] = 2.0*splineParam2[4]*t;
				gv.ddqd[4] = 2.0*splineParam2[4];
			}
			else
			{
				gv.qd[4] = gv.qd[4];
				gv.dqd[4] = 0;
			}
		}
		// Torque input for deceleration phase
		else if(gv.curTime - splineStartTime <  splineDuration + splineDuration2)
		{
			t = gv.curTime - finishTime1;		
			if (tf2[2] > gv.curTime - finishTime1)
			{
				gv.qd[2] =  splineParam0F[2] + splineParam1F[2]*t + splineParam2F[2]*pow(t,2);
				gv.dqd[2] = 2.0*splineParam2F[2]*t + splineParam1F[2];
				gv.ddqd[2] = 2.0*splineParam2F[2];
			}
			else
			{
				gv.qd[2] = gv.qd[2];
				gv.dqd[2] = 0;
				gv.ddqd[2] = 0;
			}

			if (tf2[4] > gv.curTime - finishTime1)
			{
				gv.qd[4] =  splineParam0F[4] + splineParam1F[4]*t + splineParam2F[4]*pow(t,2);
				gv.dqd[4] = 2.0*splineParam2F[4]*t + splineParam1F[4];
				gv.ddqd[4] = 2.0*splineParam2F[4];
			}
			else
			{
				gv.qd[4] = gv.qd[4];
				gv.dqd[4] = 0;
				gv.ddqd[4] = 0;
			}
		}
		else
		{
			gv.qd = gv.qd;
			gv.dqd.zero();
			gv.ddqd.zero();
			gv.dqmax[2] = 80/180.0*PI;
			gv.dqmax[4] = 80/180.0*PI;

		}
		PrVector tauPrime = gv.ddqd - gv.kp * (gv.q - gv.qd) - gv.kv * (gv.dq - gv.dqd);
		gv.tau = gv.A * tauPrime + gv.B + gv.G;
	}
}

void proj2Control(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement proj2Control
}

void proj3Control(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement proj3Control
}

// *******************************************************************
// Debug function
// *******************************************************************

void PrintDebug(GlobalVariables& gv)
{
   // Replace this code with any debug information you'd like to get
   // when you type "pdebug" at the prompt.
   printf( "This sample code prints the torque and mass\n" );
   gv.tau.display( "tau" );
   gv.A.display( "A" );
}

/********************************************************

END OF DEFAULT STUDENT FILE 

ADD HERE ALL STUDENT DEFINED AND AUX FUNCTIONS 

*******************************************************/

void FindCubicSpline(GlobalVariables& gv)
{
   splineStartTime = gv.curTime;
   splineDuration  = 0.0;
   for(int ii = 0; ii < gv.dof; ii++)
   {
      Float vDuration = fabs(gv.qd[ii] - gv.q[ii]) * 1.5 / gv.dqmax[ii];
      Float aDuration = sqrt( fabs(gv.qd[ii] - gv.q[ii]) * 6.0 / gv.ddqmax[ii] );
      splineDuration = max( splineDuration, vDuration );
      splineDuration = max( splineDuration, aDuration );
   }

   if( splineDuration < 1e-3 )
   {
      // Avoid divide-by-zero errors
      splineParam0 = gv.qd;
      splineParam2.setSize( gv.dof, true );
      splineParam3.setSize( gv.dof, true );
   }
   else
   {
      Float splineDuration2 = splineDuration * splineDuration;
      Float splineDuration3 = splineDuration * splineDuration2;
      splineParam0 = gv.q;
      splineParam2 = (gv.qd - gv.q) * (3.0 / splineDuration2);
      splineParam3 = (gv.qd - gv.q) * (-2.0 / splineDuration3);
   }
}

void FindOperationalCubicSpline(GlobalVariables& gv)
{
   splineStartTime = gv.curTime;
   splineDuration  = 0.0;

   // Find the finish time

   static const Float MIN_VMAX = 1e-4;

   if( gv.dxmax < MIN_VMAX || gv.ddxmax < MIN_VMAX || gv.wmax < MIN_VMAX )
   {
      splineDuration = 1 / MIN_VMAX;    // Avoid divide-by-zero errors
   }
   else
   {
      PrVector deltaX = gv.Einverse * ( gv.x - gv.xd );
      Float vDuration = (gv.selectLinear * deltaX).magnitude() * 1.5 / gv.dxmax;
      Float wDuration = (gv.selectAngular * deltaX).magnitude() * 1.5 / gv.wmax;
      Float aDuration =
         sqrt( (gv.selectLinear * deltaX).magnitude() * 6.0 / gv.ddxmax );
      splineDuration = max( splineDuration, vDuration );
      splineDuration = max( splineDuration, wDuration );
      splineDuration = max( splineDuration, aDuration );
   }

   if( splineDuration < 1e-3 )
   {
      // Avoid divide-by-zero errors
      splineParam0 = gv.xd;
      splineParam2.setSize( gv.xd.size(), true );
      splineParam3.setSize( gv.xd.size(), true );
   }
   else
   {
      Float splineDuration2 = splineDuration * splineDuration;
      Float splineDuration3 = splineDuration * splineDuration2;
      splineParam0 = gv.x;
      splineParam2 = (gv.xd - gv.x) * (3.0 / splineDuration2);
      splineParam3 = (gv.xd - gv.x) * (-2.0 / splineDuration3);
   }
}

void EvaluateCubicSpline( PrVector& pos, PrVector& vel, PrVector& acc, GlobalVariables& gv )
{
   float t = gv.curTime - splineStartTime;

   if( t > splineDuration )
   {
      t = splineDuration;
      pos = splineParam0 + splineParam2 * t*t + splineParam3 * t*t*t;
      vel.setSize( splineParam0.size(), true );
      acc.setSize( splineParam0.size(), true );
   }
   else
   {
      Float t2 = t * t;
      Float t3 = t * t2;
      pos = splineParam0 + splineParam2 * t2 + splineParam3 * t3;
      vel = splineParam2 * 2.0 * t + splineParam3 * 3.0 * t2;
      acc = splineParam2 * 2.0 + splineParam3 * 6.0;
   }
}

bool inv_kin( const PrVector3& pos, const PrMatrix3& rot, int elbow,
              PrVector& qOut, GlobalVariables& gv )
{
   bool q1rotated  = elbow & 0x01;
   bool elbowUp    = elbow & 0x02;
   bool negativec4 = elbow & 0x04;

   // Given the position & orientation of the end-effector, we can
   // figure out the position of the wrist
   Float wristX = pos[0] - rot[0][2] * L6;
   Float wristY = pos[1] - rot[1][2] * L6;
   Float wristZ = pos[2] - rot[2][2] * L6;

   // From the wrist position, we can find q1 by projecting the wrist
   // into the x-y plane, and finding q1 such that (x,y).(-s1,c1)=L1.

   Float distToWrist = sqrt( wristX * wristX + wristY * wristY );
   if( distToWrist < L1 )
   {
      return false;   // Wrist cannot be closer than L1 to base
   }
   if( q1rotated )
   {
      qOut[0] = atan2( wristY, wristX ) + asin( L1 / distToWrist ) - M_PI;
   }
   else
   {
      qOut[0] = atan2( wristY, wristX ) - asin( L1 / distToWrist );
   }

   // Now that we know q1, we can project the upper & lower arm into
   // the vertical plane perpendicular to the end of the first link
   // (let's call it the w-z plane), and solve the 2-D problem to find
   // q2 & q3.

   Float c1 = cos( qOut[0] );
   Float s1 = sin( qOut[0] );
   Float wristW = wristX * c1 + wristY * s1;
   Float s3 = ( wristW*wristW + wristZ*wristZ - L2*L2 - L3*L3 ) / ( 2*L2*L3 );
   if( s3 > 1 || s3 < -1 )
   {
      return false;
   }

   qOut[2] = elbowUp ? ( M_PI - asin(s3) ) : ( asin(s3) );
   Float c3 = cos( qOut[2] );
   qOut[1] = -atan2( wristZ, wristW ) + atan2( L3 * c3, L2 + L3 * s3 );

   // Now that we have q1-q3, we can find the rotation matrix of the
   // wrist.  Since we also have the rotation matrix of the
   // end-effector, we can find the rotation of the end-effector
   // relative to the wrist.
   //
   // In other words,
   //
   //  3       0 T     0
   //   R   =   R   *   R
   //  6       3       6

   Float c23 = cos( qOut[1] + qOut[2] );
   Float s23 = sin( qOut[1] + qOut[2] );

   PrMatrix3 wristRot( c1*c23, -s1, c1*s23,
                       s1*c23,  c1, s1*s23,
                         -s23,   0,    c23 );
   PrMatrix3 eeRot = wristRot.transpose() * rot;

   // Now we can use this rotation matrix to find q4 - q6

   Float c5 = eeRot[2][2];
   Float s5 = sqrt( eeRot[0][2] * eeRot[0][2] + eeRot[1][2] * eeRot[1][2] );
   if( eeRot[0][2] >= 0 && negativec4 || eeRot[0][2] < 0 && !negativec4 )
   {
      s5 = -s5;
   }
   qOut[4] = atan2( s5, c5 );

   if( qOut[4] < gv.sbound && qOut[4] > -gv.sbound )
   {
      // Singularity: we only know q4+q6.  Take the average.
      Float q46 = atan2( eeRot[1][0], eeRot[0][0] );
      qOut[3] = q46 / 2;
      qOut[5] = q46 / 2;
   }
   else if ( s5 >= 0 )
   {
      // Not a singularity, s5 is positive
      qOut[3] = atan2( eeRot[1][2],  eeRot[0][2] );
      qOut[5] = atan2( eeRot[2][1], -eeRot[2][0] );
   }
   else
   {
      // Not a singularity, s5 is negative
      qOut[3] = atan2( -eeRot[1][2], -eeRot[0][2] );
      qOut[5] = atan2( -eeRot[2][1],  eeRot[2][0] );
   }

   // Normalize all the angles to the region between gv.qmin and gv.qmax.
   // Return false if this is impossible.

   for( int ii = 0; ii < gv.dof; ii++ )
   {
      while( qOut[ii] < gv.qmin[ii] )
      {
         qOut[ii] += 2 * M_PI;
      }
      while( qOut[ii] > gv.qmax[ii] )
      {
         qOut[ii] -= 2 * M_PI;
      }
      if( qOut[ii] < gv.qmin[ii] || qOut[ii] > gv.qmax[ii] )
      {
         return false;
      }
   }

   return true;
}

void getSaturatedFprime( PrVector& fPrime, GlobalVariables& gv )
{
   static const Float MIN_KV = 1e-4;

   // Find the desired velocity (linear & angular)

   PrVector deltaX = gv.Einverse * ( gv.x - gv.xd );
   PrVector local_dxd = gv.dxd;
   for( int ii = 0; ii < gv.dxd.size(); ii++ )
   {
      local_dxd[ii] -= deltaX[ii] * gv.kp[ii] / max(gv.kv[ii], MIN_KV);
   }

   // Perform velocity saturation

   Float linearVelocity = (gv.selectLinear * local_dxd).magnitude();
   if( linearVelocity > gv.dxmax )
   {
      local_dxd *= gv.dxmax / linearVelocity;
   }

   Float angularVelocity = (gv.selectAngular * local_dxd).magnitude();
   if( angularVelocity > gv.wmax )
   {
      local_dxd *= gv.wmax / angularVelocity;
   }

   // Calculate f-prime

   fPrime = -gv.kv * (gv.dx - local_dxd);
}

void GetNonsingularSelection( PrMatrix& dest, GlobalVariables& gv )
{
   // Find rotations of frames 3 & 5
   //
   Float q1 = gv.q[0];
   Float q2 = gv.q[1];
   Float q3 = gv.q[2];
   Float q4 = gv.q[3];
   Float q5 = gv.q[4];
   float c1  = cos( q1 );
   float s1  = sin( q1 );
   float c23 = cos( q2 + q3 );
   float s23 = sin( q2 + q3 );
   float c4  = cos( q4 );
   float s4  = sin( q4 );
   float c5  = cos( q5 );
   float s5  = sin( q5 );

   PrMatrix3 R3( c1 * c23,  -c1 * s23,  -s1,
                 s1 * c23,  -s1 * s23,   c1,
                     -s23,       -c23,    0 );
   PrMatrix3 R35( c4 * c5,  -c4 * s5,  -s4,
                       s5,        c5,    0,
                  s4 * c5,  -s4 * s5,   c4 );
   PrMatrix3 R5 = R3 * R35;

   // Resize the selection matrix
   //
   int reducedDof = 6;
   if( gv.singularities & ELBOW_LOCK )
   {
      --reducedDof;
   }
   if( gv.singularities & HEAD_LOCK )
   {
      --reducedDof;
   }
   if( gv.singularities & WRIST_LOCK )
   {
      --reducedDof;
   }
   dest.setSize( reducedDof, 6, true );

   // Initialize selection matrix for ELBOW_LOCK and HEAD_LOCK
   //
   int row = 0;
   if ( ( gv.singularities & (ELBOW_LOCK | HEAD_LOCK) ) == 0 )
   {
      dest[row++][0] = 1;
      dest[row++][1] = 1;
      dest[row++][2] = 1;
   }
   else
   {
      PrVector3 x3( R3[0][0], R3[1][0], R3[2][0] );
      PrVector3 y3( R3[0][1], R3[1][1], R3[2][1] );
      PrVector3 z3( R3[0][2], R3[1][2], R3[2][2] );
      dest[row][0] = x3[0];
      dest[row][1] = x3[1];
      dest[row][2] = x3[2];
      row++;
      if ( ( gv.singularities & ELBOW_LOCK ) == 0 )
      {
         dest[row][0] = y3[0];
         dest[row][1] = y3[1];
         dest[row][2] = y3[2];
         row++;
      }
      if ( ( gv.singularities & HEAD_LOCK ) == 0 )
      {
         dest[row][0] = z3[0];
         dest[row][1] = z3[1];
         dest[row][2] = z3[2];
         row++;
      }
   }

   // Initialize selection matrix for WRIST_LOCK
   //
   if ( ( gv.singularities & WRIST_LOCK ) == 0 )
   {
      dest[row++][3] = 1;
      dest[row++][4] = 1;
      dest[row++][5] = 1;
   }
   else
   {
      PrVector3 y5( R5[0][1], R5[1][1], R5[2][1] );
      PrVector3 z5( R5[0][2], R5[1][2], R5[2][2] );
      dest[row][3] = y5[0];
      dest[row][4] = y5[1];
      dest[row][5] = y5[2];
      row++;
      dest[row][3] = z5[0];
      dest[row][4] = z5[1];
      dest[row][5] = z5[2];
      row++;
   }
}

void GetEscapeTorque( const PrVector& fPrime, PrVector& escapeTorque, GlobalVariables& gv )
{
   PrVector6 nullSpaceMotion;
   PrVector6 force;
   for( int ii = 0; ii < 6; ii++ )
   {
      static const Float MIN_KP = 0.1;
      force = fPrime / max( gv.kp[ii], MIN_KP );
   }

   // Find rotation of frame 4

   Float q1 = gv.q[0];
   Float q2 = gv.q[1];
   Float q3 = gv.q[2];
   Float q4 = gv.q[3];
   float c1  = cos( q1 );
   float s1  = sin( q1 );
   float c2  = cos( q2 );
   float c23 = cos( q2 + q3 );
   float s23 = sin( q2 + q3 );
   float c4  = cos( q4 );
   float s4  = sin( q4 );

   PrMatrix3 R4( c1*c23*c4 - s1*s4,  -c1*c23*s4 - s1*c4,  c1*s23,
                 s1*c23*c4 + c1*s4,  -s1*c23*s4 + c1*c4,  s1*s23,
                   -s23*c4,              s23*s4,             c23 );

   // Elbow lock

   if ( gv.singularities & ELBOW_LOCK )
   {
      // Assume that the radial force is proportional to the linear
      // force dot position
      Float radialForce =
         force[0] * gv.x[0] + force[1] * gv.x[1] + force[2] * gv.x[2];

      if ( radialForce > 0 )
      {
         // We want to move outward, so move q3 toward pi/2
         nullSpaceMotion[2] += radialForce * ( M_PI/2 - q3 );
      }
      else if ( L2 * c2 + L3 * s23 >= 0 )
      {
         // We want to move inward, and we're less likely to hit a
         // joint limit if we move the elbow up
         nullSpaceMotion[2] -= radialForce;
      }
      else
      {
         // We want to move inward, and we're less likely to hit a
         // joint limit if we move the elbow down
         nullSpaceMotion[2] += radialForce;
      }
   }

   // Head lock

   if ( gv.singularities & HEAD_LOCK )
   {
      // Find out where q1 should point, in order to be able to rotate
      // the end-effector to the right place
      Float desiredQ1;
      if( force[1] >= 0 )
      {
         desiredQ1 = atan2( force[1], force[0] );
      }
      else
      {
         desiredQ1 = atan2( -force[1], -force[0] );
      }

      // Find out how strong the force is in the x-y plane, and scale
      // the q1 torque accordingly.  (No need to spin q1 around if the
      // end-effector is hovering over the base.)
      Float scale = sqrt( force[0] * force[0] + force[1] * force[1] );

      nullSpaceMotion[0] += scale * ( desiredQ1 - q1 );
   }

   // Wrist lock

   if( gv.singularities & WRIST_LOCK )
   {
      // Translate the angular component of force into the x4/y4
      // plane
      PrVector3 x4( R4[0][0], R4[1][0], R4[2][0] );
      PrVector3 y4( R4[0][1], R4[1][1], R4[2][1] );

      Float torqueX4 = force[3] * x4[0] + force[4] * x4[1] + force[5] * x4[2];
      Float torqueY4 = force[3] * y4[0] + force[4] * y4[1] + force[5] * y4[2];

      // Find out where q4 should point, in order to be able to
      // apply the desired angular momentum

      Float desiredQ4;
      if( torqueY4 >= 0 )
      {
         desiredQ4 = atan2( torqueY4, torqueX4 );
      }
      else
      {
         desiredQ4 = atan2( -torqueY4, -torqueX4 );
      }

      // Find out how strong the torque vector is, projected into the
      // x-y plane, and scale the q4 torque accordingly.  (No need to
      // spin q4 around if the finger is going to stay straight.)
      Float scaleFactor = sqrt( torqueX4 * torqueX4 + torqueY4 * torqueY4 );

      nullSpaceMotion[3] += scaleFactor * ( desiredQ4 - q4 );
   }

   // Calculate escapeTorque
   //
   escapeTorque.setSize( 6 );
   for( int ii = 0; ii < 6; ii++ )
   {
      static const Float MIN_KV = 1e-4;
      Float kp  = gv.kp[ii];
      Float kv  = max( gv.kv[ii], MIN_KV );
      Float dqd = nullSpaceMotion[ii] * kp / kv;
      dqd = max( dqd, -gv.dqmax[ii] );
      dqd = min( dqd,  gv.dqmax[ii] );
      escapeTorque[ii] = -kv * ( gv.dq[ii] - dqd );
   }
}

void OpDynamics( const PrVector& fPrime, GlobalVariables& gv )
{
   // No singularities

   if( gv.singularities == 0 )
   {
      gv.tau = gv.Jtranspose * (gv.Lambda * fPrime + gv.mu + gv.p);
      return;
   }

   // Singular configuration

   static PrMatrix nonsingularSelection;
   static PrVector escapeTorque;
   static PrMatrix Ainverse;
   static PrMatrix rJacobian;
   static PrMatrix rJacobianT;
   static PrMatrix rLambdaInverse;
   static PrMatrix rLambda;
   static PrMatrix rJacobianInverseT;
   static PrMatrix identity;
   static PrMatrix nullSpaceT;

   GetNonsingularSelection( nonsingularSelection, gv );
   GetEscapeTorque( fPrime, escapeTorque, gv );
   gv.A.inverseSPD( Ainverse );
   rJacobian = nonsingularSelection * gv.J;
   rJacobian.transpose( rJacobianT );
   rLambdaInverse = rJacobian * Ainverse * rJacobianT;
   rLambdaInverse.inverseSPD( rLambda );
   rJacobianInverseT = rLambda * rJacobian * Ainverse;
   identity.setSize( 6, 6 );
   identity.identity();
   nullSpaceT = identity - rJacobianT * rJacobianInverseT;
   gv.tau = rJacobianT * rLambda * nonsingularSelection * fPrime
         + nullSpaceT * Ainverse * escapeTorque
         + gv.G;
}


#ifdef WIN32
// *******************************************************************
// XPrintf(): Replacement for printf() which calls ui->VDisplay()
// whenever the ui object is available.  See utility/XPrintf.h.
// *******************************************************************

int XPrintf( const char* fmt, ... )
{
  int returnValue;
  va_list argptr;
  va_start( argptr, fmt );

  returnValue = vprintf( fmt, argptr );

  va_end( argptr );
  return returnValue;
}
#endif //#ifdef WIN32