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

#include "OptiTrack.h"

int main(int argc, char* argv[])
{
    std::string vrpn_server_ip = "172.24.69.67:3883"; //you should not change this ip address
    std::cout << "vrpn_server_ip:" << vrpn_server_ip << std::endl;

    //The name of the objects that you want to track
    std::string target_name[] = {"target","test"};

    //computing the number of strings in the target_name
    int nbObjects = sizeof( target_name ) / sizeof( std::string );

    //initializing the optitrack object(s)
    OptiTrack *objects = new OptiTrack[nbObjects];
    for (int i=0; i<nbObjects; i++){
        objects[i].Init(vrpn_server_ip, target_name[i]);

        //if you want to use your own coordinate system T, you could load it here
        //T is a homogeneous matrix from OptiTrack coordinate system to your coordinate system
        objects[i].loadCalibrationMatrix("VisionCalib.txt");
    }

    bool b_useCalibration = true;
    double currentPosition[3];
    double **currentRotationMatrix;
    currentRotationMatrix = new double*[3];
    for (int i=0; i<3; i++)
        currentRotationMatrix[i] = new double[3];

    while(true)
    {
        for (int i=0; i<nbObjects; i++)
        {
            if (objects[i].IsEnabled()){
                //reading new values, if there is any
                objects[i].Update();

                // Getting the object position
                objects[i].GetPosition(currentPosition, b_useCalibration);

                // Getting the object orientation
                objects[i].GetRotationMatrix(currentRotationMatrix, b_useCalibration);

                //printing
                std::cout << "Object name: " << objects[i].GetObjectName() << std::endl;
                std::cout << "Position:    [" << currentPosition[0] << "," << currentPosition[1] << "," << currentPosition[2] << "]" << std::endl;

                std::cout << "Orientation: [" << currentRotationMatrix[0][0] << "," << currentRotationMatrix[0][1] << "," << currentRotationMatrix[0][2] << std::endl;
                std::cout << "              " << currentRotationMatrix[1][0] << "," << currentRotationMatrix[1][1] << "," << currentRotationMatrix[1][2] << std::endl;
                std::cout << "              " << currentRotationMatrix[2][0] << "," << currentRotationMatrix[2][1] << "," << currentRotationMatrix[2][2] << "]" << std::endl;
            }
        }

        vrpn_SleepMsecs(10);
    }
	return 0;
}



