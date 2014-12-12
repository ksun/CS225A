#include <Winsock.h>
#include <windows.h>
#include "robotCom.h"

//you will need to change PrNetworkDefn and Robot.cpp based on the 
//QNX computer you are using
//Copy cs225a.h from your cs225asim directory
#include <iostream>
#include <tchar.h>
#include "prvector.h"
#include "prvector3.h"
#include "prmatrix.h"
#include "prmatrix3.h"
#include <fstream>
#include "param.h"
#include <string.h>
#include <stdlib.h>
#include "math.h"

#define MAX_NUM_OF_NOTES 128	//this is the number of notes in the note list
#define NUM_OF_XYLOPHONE_NOTES 12
#define NUM_JOINT_ANGLES 6
#define X_POS_ORIENTATION 7
#define PRE_IMPACT_OFFSET 25
#define PI 3.14159265


static int milliSecsPerNote = 0;


/*Fills impactPositionsXPos with the correct (x,y,z) coordinates of each note and the correct orientation of the
 *end effector
 */
void calculateImpactPositions(float impactPositionsXPos[][X_POS_ORIENTATION], float leftXPos[], float rightXPos[])
{
	/*Vector from the right hand side to the left hand side of the xylaphone*/
	float xyzPos[3];
	for(int i=0; i<3; i++)
	{
		xyzPos[i] = rightXPos[i] - leftXPos[i];
	}

	/*Now we interperolate all the notes between the two endpoints */
	for(int i=0; i < NUM_OF_XYLOPHONE_NOTES; i++)
	{
		for(int j=0; j<3; j++)
		{
			impactPositionsXPos[i][j] = leftXPos[j] + ((float)i)*xyzPos[j]/((float)(NUM_OF_XYLOPHONE_NOTES-1.0));
		}
	}

	/*The orientation vector is calculated how ??? For now we assume that it is the same orientation that
	 *the left setting had
	 */
	for(int i=0; i < NUM_OF_XYLOPHONE_NOTES; i++)
	{
		for(int j=3; j<7; j++)
		{
			impactPositionsXPos[i][j] = leftXPos[j];
		}
	}
}

/* fill in this code when we have the working control.cpp */
bool inv_kin(const PrVector3& pos, const PrMatrix3& rot, int elbow, PrVector& qOut)
{
	qOut.zero();
	PrVector3 wrist_in_base;
	PrVector3 EE_in_wrist;
	wrist_in_base.zero();
	wrist_in_base[2] = L6;

	//This finds a position vector to the wrist point from end effector
	wrist_in_base = rot*wrist_in_base;

	//this finds the position vector to the wrist relative to robot origin
	wrist_in_base = -1*wrist_in_base + pos;
	//theta1=arctan(y_wrist/x_wrist)

	int joint1flag;
	if (elbow&0x01) joint1flag=1;
	else joint1flag=-1;

	int joint2flag;
	if (elbow&0x02) joint2flag=1;
	else joint2flag=-1;

	float wx=wrist_in_base[0];
	float wy=wrist_in_base[1];
	float wz=wrist_in_base[2];
	//qOut[0]=(float)joint1flag*(atan2(wy,wx)+acos(L1/(sqrt(wx*wx+wy*wy))));
	qOut[0]=atan2((-1*(float)joint1flag*wy*sqrt(wx*wx+wy*wy-L1*L1)-wx*L1),(-1*(float)joint1flag*wx*sqrt(wx*wx+wy*wy-L1*L1)+wy*L1));

	float R=sqrt(wx*wx+wy*wy+wz*wz-L1*L1);
	float r=sqrt(wx*wx+wy*wy-L1*L1);
	float sinalpha=-wz/R;
	float cosalpha=-(float)joint1flag*r/R;
	float cosbeta=(L2*L2+R*R-L3*L3)/(2*L2*R);
	float sinbeta=sqrt(1-cosbeta*cosbeta);

	qOut[1]=atan2(sinalpha*cosbeta+(float)joint1flag*(float)joint2flag*cosalpha*sinbeta,cosalpha*cosbeta-(float)joint1flag*(float)joint2flag*sinalpha*sinbeta);

	float cosphi = (L2*L2 + L3*L3 - R*R)/(2*L2*L3);
	float sinphi = (float)joint1flag*(float)joint2flag*sqrt(1-cosphi*cosphi);
	float sinbeta1 = 1;
	float cosbeta1 = 0;

	qOut[2]=atan2(sinphi*cosbeta1-cosphi*sinbeta1,cosphi*cosbeta1+sinphi*sinbeta1);

	int joint4flag;
	if (elbow&0x04) joint4flag=-1;
	else joint4flag=1;

	float dotProduct = EE_in_wrist[2]*L6;
	float norm=sqrt(EE_in_wrist[0]*EE_in_wrist[0]+EE_in_wrist[1]*EE_in_wrist[1]+EE_in_wrist[2]*EE_in_wrist[2]);
	
	//from the notes
	//unit vectors n, s, a correspond to the endeffector axis in the base frame
	PrVector3 n_base;
	n_base.zero();
	n_base[0] = 1;
	n_base = rot*n_base;

	PrVector3 s_base;
	s_base.zero();
	s_base[1] = 1;
	s_base = rot*s_base;

	PrVector3 approachVec;
	approachVec.zero();
	approachVec[2] = 1;

	//approachVec should now be a unit vector point in the same direction as the end effector, in the base frame
	approachVec = rot*approachVec;

	//now we want the unit vector of z3 with respect to the base frame.
	PrVector3 z3_in_base;
	z3_in_base[0]=-.5*sin(qOut[0]-qOut[1]-qOut[2])+.5*sin(qOut[0]+qOut[1]+qOut[2]);
	z3_in_base[1]=-.5*cos(qOut[0]-qOut[1]-qOut[2])+.5*cos(qOut[0]+qOut[1]+qOut[2]);
	z3_in_base[2]=cos(qOut[1]+qOut[2]);


	float omega = 0.0;
	PrVector3 omega_tmp = z3_in_base.cross(approachVec);
	float normOmegaS = sqrt(omega_tmp[0]*omega_tmp[0] + omega_tmp[1]*omega_tmp[1] + omega_tmp[2]*omega_tmp[2]);
	PrVector3 omegaTmpNormalized;
	for (int i=0; i<omega_tmp.size(); i++)
		omegaTmpNormalized[i] = omega_tmp[i]/normOmegaS;

	float omegaCheck = s_base.dot(omega_tmp);

	if (abs(omegaCheck) > 1e-3) 
		omega = s_base.dot(omegaTmpNormalized);
	else if (abs(omegaCheck) <= 1e-3)
		omega = n_base.dot(omegaTmpNormalized);
	if (abs(normOmegaS) <=1e-3)
		omega = 0; //degenerate case

	float M_value = 0;
	if (omega >= 0)
		M_value = 1;
	else if (omega < 0)
		M_value = -1;

	M_value=((float)joint4flag)*M_value;

	qOut[3] = atan2(M_value*(cos(qOut[0])*approachVec[1] - sin(qOut[0])*approachVec[0]), M_value*(cos(qOut[0])*cos(qOut[1]+qOut[2])*approachVec[0] + sin(qOut[0])*cos(qOut[1]+qOut[2])*approachVec[1] - sin(qOut[1]+qOut[2])*approachVec[2]));

	qOut[4] = atan2((cos(qOut[0])*cos(qOut[1]+qOut[2])*cos(qOut[3])-sin(qOut[0])*sin(qOut[3]))*approachVec[0]+(sin(qOut[0])*cos(qOut[1]+qOut[2])*cos(qOut[3])+cos(qOut[0])*sin(qOut[3]))*approachVec[1]-cos(qOut[3])*sin(qOut[1]+qOut[2])*approachVec[2], cos(qOut[0])*sin(qOut[1]+qOut[2])*approachVec[0]+sin(qOut[0])*sin(qOut[1]+qOut[2])*approachVec[1]+cos(qOut[1]+qOut[2])*approachVec[2]);

	qOut[5] = atan2((-sin(qOut[0])*cos(qOut[3])-cos(qOut[0])*cos(qOut[1]+qOut[2]))*n_base[0]+(cos(qOut[0])*cos(qOut[3])-sin(qOut[0])*cos(qOut[1]+qOut[2]))*n_base[1]+(sin(qOut[3])*sin(qOut[1]*qOut[2]))*n_base[2],(-sin(qOut[0])*cos(qOut[3])-cos(qOut[0])*cos(qOut[1]+qOut[2]))*s_base[0]+(cos(qOut[0])*cos(qOut[3])-sin(qOut[0])*cos(qOut[1]+qOut[2]))*s_base[1]+(sin(qOut[3])*sin(qOut[1]*qOut[2]))*s_base[2]);
	
	/*Now we convert everything to degrees */
	for(int i=0; i<NUM_JOINT_ANGLES; i++)
		qOut[i] = (180.0/PI)*qOut[i];
	
	//make sure qOut is within joint limits
	float qmin[NUM_JOINT_ANGLES] = {-160, -223, -40, -130, -100, -266};
	float qmax[NUM_JOINT_ANGLES] = {160, 43, 240, 180, 100, 266};
	for (int i=0; i < qOut.size(); i++) {
		if (qOut[i] > qmax[i] || qOut[i] < qmin[i])
			return false;
	}

    return true;
}

void calculateJPosForEachNoteImpact(float impactPositionsXPos[][X_POS_ORIENTATION], float impactPositionsJPos[][NUM_JOINT_ANGLES])
{
	PrVector3 curPos;
	PrMatrix3 curRot;
	int elbow = 0;
	PrVector jOut;
	jOut.setSize(6);

	/*Here we calculate the rotation matrix by crossing the vector that spans the xylophone with the z-axis unit
	 *vector. 
	*/
	PrVector3 y_in_base;
	y_in_base.zero();
	y_in_base[1] = 1;
	PrVector3 xylophoneRadius;
	//xylophoneRadius is the vector from right hand side to the left hand side
	for(int i=0; i<3; i++)
		xylophoneRadius[i] = impactPositionsXPos[NUM_OF_XYLOPHONE_NOTES-1][i] - impactPositionsXPos[0][i];
	float theta = acos(y_in_base.dot(xylophoneRadius)/(y_in_base.magnitude()*xylophoneRadius.magnitude()));

	curRot.zero();
	curRot[0][0] = 0;
	curRot[0][1] = -sin(theta);
	curRot[0][2] = cos(theta);
	curRot[1][0] = 0;
	curRot[1][1] = cos(theta);
	curRot[1][2] = sin(theta);
	curRot[2][0] = 1;
	curRot[2][1] = 0;
	curRot[2][2] = 0;

	for(int i=0; i<NUM_OF_XYLOPHONE_NOTES; i++)
	{
		for(int j=0; j<3; j++)
		{
			curPos[j] = impactPositionsXPos[i][j];
		}
		
		/*now we need to translate the 4-vector orientation into a rotation matrix*/
		//????
		inv_kin(curPos, curRot, elbow, jOut);
		for(int k=0; k<NUM_JOINT_ANGLES; k++)
		{
			impactPositionsJPos[i][k] = jOut[k];
		}
	}
}

/*The pre-impact position is the position the arm should be at before it tries to strike the xylophone
 *to play the desired note
 */
void calculateJPosForEachNotePrePosition(float impactPositionsJPos[][NUM_JOINT_ANGLES], float preImpactPositionsJPos[][NUM_JOINT_ANGLES])
{
	for(int i=0; i < NUM_OF_XYLOPHONE_NOTES; i++)
	{
		for(int j=0; j < NUM_JOINT_ANGLES; j++)
		{
			preImpactPositionsJPos[i][j] = impactPositionsJPos[i][j];
		}
	}

	/*Here we raise the wrist up 20 degrees so it can gain momentum before hitting the xylophone pad */
	float parity = 1.0;
	for(int i=0; i<NUM_OF_XYLOPHONE_NOTES; i++)
	{
		if(impactPositionsJPos[i][1] >= -40 && impactPositionsJPos[i][1] <= 45)
		{
			if(impactPositionsJPos[i][3] >= -20 && impactPositionsJPos[i][3] <= 20)
				parity = -1.0;
			if(impactPositionsJPos[i][3] >= 160 && impactPositionsJPos[i][3] <= 180)
				parity = 1.0;
		}
		else if(impactPositionsJPos[i][1] >= -225 && impactPositionsJPos[i][1] <= -150)
		{
			if(impactPositionsJPos[i][3] >= -20 && impactPositionsJPos[i][3] <= 20)
				parity = 1.0;
			if(impactPositionsJPos[i][3] >= 160 && impactPositionsJPos[i][3] <= 180)
				parity = -1.0;
		}

		preImpactPositionsJPos[i][4] = impactPositionsJPos[i][4] + (parity * PRE_IMPACT_OFFSET);
	}
}


int _tmain(int argc, _TCHAR* argv[])
{
	RobotCom* myRobot = new RobotCom(); // Connecting to the QNX

	/*for now lets say the robot can only play note at a time, each index of noteList corresponding to a quarter note */
	int noteList[MAX_NUM_OF_NOTES];
	int numOfNotes = 0;
	float impactPositionsXPos[NUM_OF_XYLOPHONE_NOTES][7];
	float impactPositionsJPos[NUM_OF_XYLOPHONE_NOTES][6];
	float preImpactPositionsJPos[NUM_OF_XYLOPHONE_NOTES][6];
	float leftXPos[7];
	float rightXPos[7];
	float noteStartTime;
	float noteEndTime;


	/*Read in the music file to play*/
	std::ifstream inFile;
	
	inFile.open("music.txt");
    if (!inFile) {
		std::cout << "Unable to open file";
        exit(1); // terminate with error
    }

	/*Read in the time in between playing each note */
	if(!inFile.eof())
		inFile >> milliSecsPerNote;

	while (!inFile.eof()) {
		inFile >> noteList[numOfNotes];
		++numOfNotes;
		if(numOfNotes>MAX_NUM_OF_NOTES)
			break;
	}
	inFile.close();

	float *q;
	float temp_q[NUM_JOINT_ANGLES];
	//jtrack robot to {0,0,0,0,0,0}
	for(int i=0;i < NUM_JOINT_ANGLES;i++)
	{
		temp_q[i] = 0.0;
	}
	q = temp_q;

	myRobot->control( JTRACK, q, 6 );
	Sleep(1000);

	myRobot->control(FLOATMODE, q, 6);
	std::cout << "Press enter when end-effector is on the left most note\n";
	getchar();
	myRobot->getStatus(GET_IPOS, leftXPos);
	std::cout << "Press enter when the end-effector is on the right most note\n";
	getchar();

	myRobot->getStatus(GET_IPOS, rightXPos);

	calculateImpactPositions(impactPositionsXPos, leftXPos, rightXPos);
	calculateJPosForEachNoteImpact(impactPositionsXPos, impactPositionsJPos);
	calculateJPosForEachNotePrePosition(impactPositionsJPos, preImpactPositionsJPos);

	//Sleep for 2 seconds before we start playing
	Sleep(2000);

	while( true )
	{
		// read robot status
		
		// read camera information

		// make motion command

		// send the command to the robot

		/* The plan of action is:
		 * 1. Move to the impact location
		 * 2. Immediately move to the next note's pre-impact location
		 * 3. Wait for the milliSecsPerNote time to expire
		 */
		for (int i=0; i<numOfNotes; i++) {
			switch(noteList[i])
			{
				case 10:
					q = impactPositionsJPos[0];
					break;
				case 20:
					q = impactPositionsJPos[1];
					break;
				case 30:
					q = impactPositionsJPos[2];
					break;
				case 40:
					q = impactPositionsJPos[3];
					break;
				default:
					q = temp_q;
					break;
			}
			myRobot->getStatus(GET_CURTIME, &noteStartTime);
			myRobot->control(JTRACK, q, 6);

			/*add in code here for pausing until we are really close to the impact site */
			Sleep(500);
			
			if(i < (numOfNotes-1))
			{
				switch(noteList[i+1])
				{
					case 10:
						q = preImpactPositionsJPos[0];
						break;
					case 20:
						q = preImpactPositionsJPos[1];
						break;
					case 30:
						q = preImpactPositionsJPos[2];
						break;
					case 40:
						q = preImpactPositionsJPos[3];
						break;
					default:
						q = temp_q;
						break;
				}
				myRobot->control(JTRACK, q, 6);
			}

			myRobot->getStatus(GET_CURTIME, &noteEndTime);
			Sleep(milliSecsPerNote - (noteEndTime - noteStartTime));
		}
	}

	delete myRobot;

	return 0;
}