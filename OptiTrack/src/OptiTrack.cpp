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


#include "OptiTrack.h"

#define NORM_TOLERANCE 0.01
#define UNIT 1

void VRPN_CALLBACK track_target (void *, const vrpn_TRACKERCB t);

void OptiTrack::Init(std::string connec_nm, std::string target_name)
{
    objectName = target_name;
    connection = vrpn_get_connection_by_name(connec_nm.c_str());
    tracker = new vrpn_Tracker_Remote(objectName.c_str(), connection);
    this->tracker->register_change_handler(this, track_target);

    position = new double[3];
    quaternion = new double[4];
    rotationMatrix = new double*[3];
    for (int i=0; i<3; i++)
        rotationMatrix[i] = new double[3];
    b_tracking = true;
    std::cout << "object '"<< objectName << "' was initialized." << std::endl;

    for (int i=0; i<3; i++)
        trackingOffsetPosition[i] = 0.0;

    objectFunctionality = OptiTrack::None;
}


void OptiTrack::Update()
{
    this->tracker->mainloop();
    this->connection->mainloop();
}

void OptiTrack::MatrixFromQuat(double qx, double qy, double qz, double qw, double** rotmat)
{
    rotmat[0][0] = qx*qx + qw*qw - qz*qz - qy*qy ;
    rotmat[0][1] = 2*(qx*qy - qz*qw);
    rotmat[0][2] = 2*(qx*qz + qy*qw);
    rotmat[1][0] = 2*(qx*qy + qz*qw);
    rotmat[1][1] = qy*qy - qx*qx + qw*qw - qz*qz ;
    rotmat[1][2] = 2*(qy*qz - qx*qw);
    rotmat[2][0] = 2*(qx*qz - qy*qw);
    rotmat[2][1] = 2*(qy*qz + qx*qw);
    rotmat[2][2] = qz*qz- qy*qy - qx*qx + qw*qw ;
}

void OptiTrack::RegisterCurrentData(const _vrpn_TRACKERCB &tracker)
{
    for (int i=0; i<3; i++)
    {
        position[i] =  tracker.pos[i];
        quaternion[i] =  tracker.quat[i];
    }
    quaternion[3] =  tracker.quat[3];

    MatrixFromQuat(quaternion[0], quaternion[1], quaternion[2], quaternion[3], rotationMatrix);
}

void OptiTrack::GetPosition(double* currentPosition, bool b_useCalibration)
{
    for (int i=0; i<3; i++)
        currentPosition[i] =  position[i];

    if (b_isCalibrated && b_useCalibration)
        CalibratePosition(currentPosition);
}

void OptiTrack::GetRotationMatrix(double** currentRotationMatrix, bool b_useCalibration)
{
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            currentRotationMatrix[i][j] =  rotationMatrix[i][j];

    if (b_isCalibrated && b_useCalibration)
        CalibrateOrientation(currentRotationMatrix);
}

void OptiTrack::Print()
{
    std::cout << "Object name: " << objectName << std::endl;
    std::cout << "Position:    [" << position[0] << "," << position[1] << "," << position[2] << "]" << std::endl;

    std::cout << "Orientation: [" << rotationMatrix[0][0] << "," << rotationMatrix[0][1] << "," << rotationMatrix[0][2] << std::endl;
    std::cout << "              " << rotationMatrix[1][0] << "," << rotationMatrix[1][1] << "," << rotationMatrix[1][2] << std::endl;
    std::cout << "              " << rotationMatrix[2][0] << "," << rotationMatrix[2][1] << "," << rotationMatrix[2][2] << "]" << std::endl;
}

bool OptiTrack::loadCalibrationMatrix(const char *filename)
{
    int dummy;
    FILE *calib= fopen(filename, "r+");
    double norm = 0.0;

    if(!calib)
    {
        std::cout<<"ERROR: Calibration file not found!"<<std::endl;
        return b_isCalibrated;
    }

    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            dummy = fscanf(calib,"%lf", &(calibrationOffsetRotation[i][j]));
        }
    }

    for(int j=0; j<3; j++)
    {
        for(int i=0; i<3; i++)
        {
            norm += calibrationOffsetRotation[i][j]*calibrationOffsetRotation[i][j];
        }
        if(norm<UNIT-NORM_TOLERANCE || norm>UNIT+NORM_TOLERANCE)
        {
            std::cout<<"Rotation Matrix Not normalized or Invalid.Calibration not loaded."<<std::endl;
            return b_isCalibrated;
        }
        norm = 0.0;
    }

    for(int i=0; i<3; i++)
        dummy = fscanf(calib, "%lf", &(calibrationOffsetPosition[i]));
    fclose(calib);
    std::cout<<"Calibration file: "<<filename<<" loaded!"<<std::endl;

    char txt[256];
    sprintf(txt,"Position: [% 1.5f, % 1.5f, % 1.5f]",calibrationOffsetPosition[0],calibrationOffsetPosition[1],calibrationOffsetPosition[2]);
    std::cout << txt << std::endl;

    sprintf(txt,"Rotation: [% 1.5f, % 1.5f, % 1.5f \n           % 1.5f, % 1.5f, % 1.5f \n           % 1.5f, % 1.5f, % 1.5f]",calibrationOffsetRotation[0][0],calibrationOffsetRotation[0][1],calibrationOffsetRotation[0][2]
            ,calibrationOffsetRotation[1][0],calibrationOffsetRotation[1][1],calibrationOffsetRotation[1][2],calibrationOffsetRotation[2][0],calibrationOffsetRotation[2][1]
            ,calibrationOffsetRotation[2][2]);
    std::cout << txt << std::endl;

    b_isCalibrated = true;
    return b_isCalibrated;
}

void OptiTrack::CalibratePosition(double* position)
{
    double x = position[0];
    double y = position[1];
    double z = position[2];

    position[0] = calibrationOffsetRotation[0][0]*x + calibrationOffsetRotation[0][1]*y + calibrationOffsetRotation[0][2]*z + calibrationOffsetPosition[0] + trackingOffsetPosition[0];
    position[1] = calibrationOffsetRotation[1][0]*x + calibrationOffsetRotation[1][1]*y + calibrationOffsetRotation[1][2]*z + calibrationOffsetPosition[1] + trackingOffsetPosition[1];
    position[2] = calibrationOffsetRotation[2][0]*x + calibrationOffsetRotation[2][1]*y + calibrationOffsetRotation[2][2]*z + calibrationOffsetPosition[2] + trackingOffsetPosition[2];

    //	cout<<"\nCalibrated Positions:\t"<<xtemp<<"\t"<<ytemp<<"\t"<<ztemp<<endl;
}

void OptiTrack::CalibrateOrientation(double** orient)
{
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
        {
            _tmpRotMat[i][j] = 0;

            for(int k=0;k<3;k++)
                _tmpRotMat[i][j] += calibrationOffsetRotation[i][k]*orient[k][j];
        }

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            orient[i][j] = _tmpRotMat[i][j];

}

//== Tracker Position/Orientation Callback ==--
void VRPN_CALLBACK track_target (void *userdata, const vrpn_TRACKERCB t)
{
    OptiTrack	*optiTrack = (OptiTrack*)userdata;	// Type-cast to the right type

    optiTrack->RegisterCurrentData(t);
}




