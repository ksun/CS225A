/**************************************************************************
***    Copyright (c) 2014 S. Mohammad Khansari, Stnford Robotics,       ***
***                      Stanford University, USA                       ***
***************************************************************************
*
* The program is free for non-commercial academic use, but WITHOUT ANY
* WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE. The author DOES NOT take any responsibility for
* this software.
*
* This wrapper relies on vrpn software, which is provided by Georgia Tech Research
* Corporation. You NEED to refer to vrpn package for the license information.
*
* Please send your feedbacks or questions to:
*                           khansari_at_cs.stanford.edu
***************************************************************************/

/*
# Copyright (c) 2011, Georgia Tech Research Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

## author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)
## author Chih-Hung Aaron King (Healthcare Robotics Lab, Georgia Tech.)
*/

#ifndef OPTITRACK_H_
#define OPTITRACK_H_

//#include <geometry_msgs/TransformStamped.h>
#include <stdio.h>
#include <math.h>
#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>
#include <iostream>
//#include <LinearMath/btQuaternion.h>

/*
class TargetState{
    public:
        geometry_msgs::TransformStamped target;
};
*/

class OptiTrack {
    private:
        vrpn_Connection *connection;
        vrpn_Tracker_Remote *tracker;

    private:
        bool b_tracking;
        bool b_isInit;
        bool b_isCalibrated;

        std::string objectName;
        double* position;
        double** rotationMatrix;
        //btQuaternion quaternion;
        double* quaternion;

        double calibrationOffsetPosition[3];
        double calibrationOffsetRotation[3][3];

        double _tmpRotMat[3][3];

    public:
        OptiTrack(){}

        void Init(std::string connec_nm, std::string target_name);

        void Update();

        void RegisterCurrentData(const _vrpn_TRACKERCB &tracker);

        void GetPosition(double* currentPosition, bool b_useCalibration = true);

        void GetRotationMatrix(double** currentRotationMatrix, bool b_useCalibration = true);

        std::string GetObjectName(){return objectName;}

        void EnableObjectTracking(bool enable){b_tracking = enable;}

        bool IsEnabled(){return b_tracking;}

        void MatrixFromQuat(double qx, double qy, double qz, double qw, double** rotmat);

        void Print();

        bool loadCalibrationMatrix(const char *filename);

    public:

        double trackingOffsetPosition[3]; //if you want to intentionally add some offset to the estimated position

        enum ObjectFunctionality{None, Target, EndEffector, Obstcle};

        ObjectFunctionality objectFunctionality;

private:
        void CalibratePosition(double* position);

        void CalibrateOrientation(double** orient);
};

#endif //OPTITRACK_H_
