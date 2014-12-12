/*
 * Copyright (C) 2012 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * 
 *  streamnode.cpp
 *
 *  Created on : Aug 14, 2012
 *  Author     : Ashwini Shukla
 *  Email      : ashwini.shukla@epfl.ch
 *  Website    : lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */


#include "VisionPkg/OptiTrack.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "tf/tf.h"


int main(int argc, char** argv)
{

	ros::init(argc, argv, "optitrack_tf_broadcaster");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	btVector3 tmpV3;
	btQuaternion tmpQuat;

	btMatrix3x3 btm;
	OptiTrack *tracker;

	tf::StampedTransform robotTf;
	tf::StampedTransform visionTf;
	robotTf.setIdentity();
	robotTf.child_frame_id_ = "robot";
	robotTf.frame_id_ = "vision";

	visionTf.setIdentity();
	visionTf.frame_id_ = "world";
	visionTf.child_frame_id_ = "vision";

	double pos[3];
	double orient[3][3];

	static tf::TransformBroadcaster br;

	std::string localip, objlist;
    bool exists = nh_private.getParam("local_ip", localip);

	if (!exists) {
		ROS_FATAL("You have to define local_ip!");
		return -1;
	}

	int publish_frequency = 250;
	nh_private.getParam("publish_frequency", publish_frequency);

	bool bUseThread = false;
    nh_private.getParam("use_thread", bUseThread);

	tracker = new OptiTrack();
	int ret;
	if((ret = tracker->Init(localip.c_str(), bUseThread)) <= 0)
	{
		if(ret == -1)
			ROS_FATAL("Cannot open socket. Check local ip!");
		else if(ret == -2)
			ROS_FATAL("Cannot receive data. Check windows server!");
		return -1;
	}
	tracker->enableWarnings(false);

	std::string calibfile; bool bUseCalibration = false;

	if(nh_private.getParam("calib_file", calibfile))
	{
		FILE* file = NULL;
		file = fopen(calibfile.c_str(), "r");

		if(file == NULL)
		{
			ROS_WARN_STREAM("Calibration file \""<<calibfile<<"\" not found. Robot frame will be identity.");
		}
		else
		{
			int dum;
			for(int i=0;i<3;i++)
				for(int j=0;j<3;j++)
					dum = fscanf(file, "%lf", &(orient[i][j]));

			btm.setValue(orient[0][0], orient[0][1], orient[0][2],
					orient[1][0], orient[1][1], orient[1][2],
					orient[2][0], orient[2][1], orient[2][2]);



			for(int j=0;j<3;j++)
				dum = fscanf(file, "%lf", &(pos[j]));

			tmpV3.setValue(pos[0], pos[1], pos[2]);

			btm = btm.transpose();
			tmpV3 = btm*tmpV3;
			tmpV3 *= -1.0;

			btm.getRotation(tmpQuat);
			robotTf.setOrigin(tmpV3);
			robotTf.setRotation(tmpQuat);

			bUseCalibration = true;
		}
	}

	std::vector<std::string> obj_names;
	exists &= nh_private.getParam("obj_list", objlist);


	if(!exists)				// If parameter obj_list does not exist, track all objects available
	{
		obj_names = tracker->GetRBodyNameList();
	}
	else					// If parameter obj_list exists, retrieve object names from the delimited list string
	{

		std::string delimiters = " ,;:";
		size_t current;
		size_t next = -1;
		do
		{
			current = next + 1;
			next = objlist.find_first_of( delimiters, current );
			obj_names.push_back(objlist.substr( current, next - current ));
		}
		while (next != string::npos);
	}

	std::string tmp = "";
	unsigned int i;
	for(i=0;i<obj_names.size()-1;i++)
		tmp = tmp + obj_names[i] + " ; ";

	tmp = tmp + obj_names[i];
	unsigned int num_obj = obj_names.size();

	ROS_INFO("The following parameters will be used");
	ROS_INFO_STREAM("publish_frequency = " << publish_frequency << " Hz");
	ROS_INFO_STREAM("local_ip = " << localip);
	ROS_INFO_STREAM("Tracking " << num_obj << " objects ==> " <<tmp);
	ROS_INFO_STREAM("use_thread = " << bUseThread);
	ROS_INFO_STREAM("use_calibration = " << bUseCalibration);

	for(i=0;i<num_obj;i++)
		tracker->enableRBody(obj_names[i].c_str(), true);

	ros::Rate r(publish_frequency);

	std::vector<tf::StampedTransform> transforms;
	if(num_obj)
		transforms.resize(num_obj);

	for(i=0;i<num_obj;i++)
	{
		transforms[i].child_frame_id_ = obj_names[i];
		transforms[i].frame_id_ = "vision";
		transforms[i].setIdentity();
	}

	transforms.push_back(robotTf);
	transforms.push_back(visionTf);

	ROS_INFO("TF Broadcaster started");
	while(ros::ok())
	{
	  ros::spinOnce();

		if(tracker->Update() < 0)
		{
			ROS_ERROR_STREAM_THROTTLE(1, "Tracker update failed");
			break;
		}

		for(i=0;i<num_obj;i++)
		{
			if(!tracker->getRBodyPosition(pos, obj_names[i].c_str()) || !tracker->getRBodyOrientation(orient, obj_names[i].c_str()))
				ROS_WARN_STREAM_THROTTLE(1, "Object " << obj_names[i] << " not detected");
			else
			{
				btm.setValue(orient[0][0], orient[0][1], orient[0][2],
						orient[1][0], orient[1][1], orient[1][2],
						orient[2][0], orient[2][1], orient[2][2]);

				tmpV3[0] = pos[0];
				tmpV3[1] = pos[1];
				tmpV3[2] = pos[2];

				btm.getRotation(tmpQuat);
				transforms[i].setOrigin(tmpV3);
				transforms[i].setRotation(tmpQuat);

			}
			transforms[i].stamp_ = ros::Time::now();
		}
		transforms[num_obj].stamp_ = ros::Time::now();
		transforms[num_obj+1].stamp_ = ros::Time::now();

		br.sendTransform(transforms);
		r.sleep();
	}

	ROS_INFO_STREAM("Stopping node");
	delete tracker;
	nh.shutdown();
	nh_private.shutdown();
	return 0;
}
