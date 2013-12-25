/******************************************************************************
* Copyright (c) 2011
* Locomotec
*
* Author:
* Sebastian Blumenthal
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and BSD license. The dual-license implies that users of this
* code may choose which terms they prefer.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Locomotec nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 2.1 of the
* License, or (at your option) any later version or the BSD license.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL and BSD license along with this program.
*
******************************************************************************/

#include <iostream>
#include <assert.h>

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "brics_actuator/CartesianWrench.h"

#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <math.h>

#include "ros/ros.h"
#include "brics_actuator/JointPositions.h"
#include "sensor_msgs/JointState.h"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include "std_msgs/String.h"

double joint[5];
double lastJoint[5];		
double gripperr = 0;
double gripperl = 0;

double jointMax[] = {5.840139, 2.617989, -0.0157081, 3.42919, 5.641589};
double jointMin[] = {0.01006921, 0.01006921, -5.0264, 0.0221391, 0.11062};
double gripperMax = 0.0115;
double gripperMin = 0;

double jointDelta[5];
double gripperDelta = (gripperMax - gripperMin) * 0.02;	

double jointHome[] = {0.01007,0.01007,-0.15709,0.02214,0.1107};
double jointCamera[] = {3.0,0.5,-0.9,0.1,3.0};
double jointObject[] = {3.04171,0.63597,-1.017845,0.36284,2.876194};
double jointGrasp[] = {3.04171,2.04427,-1.5189129,2.5434289757,2.8761944};
double jointInitialize[] = {0.01007,.635971,-1.91989,1.04424,2.87619};

using namespace std;
/*
bool commandMatch()
{
	cout << "Comparing values..." << endl;	
	
	bool match = false;
	int matchCounter;

	matchCounter = 0;

	for(int i = 0; i < 5; i++)
	{
		if(	(joint[i] >= lastJoint[i] - jointDelta[i]) && 
			(joint[i] <= lastJoint[i] + jointDelta[i])	)
		{

			matchCounter++;
			//cout << "matchCounter: " << matchCounter << endl;
		}
	}	

	if(matchCounter == 5) match = true;

	return match;
}
*/
void position_listener(const sensor_msgs::JointState::ConstPtr& msg)
{
	if(msg->name[0] == "arm_joint_1")
	{
		joint[0] = msg->position[0];
		joint[1] = msg->position[1];
		joint[2] = msg->position[2];
		joint[3] = msg->position[3];
		joint[4] = msg->position[4];

		gripperl = msg->position[5];
		gripperr = msg->position[6];
	}
}

char getch(void)
{
	char ch;
	struct termios oldt;
	struct termios newt;
	tcgetattr(STDIN_FILENO, &oldt); /*store old settings */
	newt = oldt; /* copy old settings to new settings */
	newt.c_lflag &= ~(ICANON | ECHO); /* make one change to old settings in new settings */
	tcsetattr(STDIN_FILENO, TCSANOW, &newt); /*apply the new settings immediatly */
	ch = getchar(); /* standard getchar call */
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt); /*reapply the old settings */
	return ch; /*return received char */
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "youbot_arm_control_test");
	ros::NodeHandle n;
	ros::Publisher armPositionsPublisher;
	ros::Publisher gripperPositionPublisher;
	ros::Subscriber armPositionSubscriber;

	armPositionsPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
	gripperPositionPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/gripper_controller/position_command", 1);
	armPositionSubscriber = n.subscribe("joint_states", 1000, position_listener);

	std::fill_n(joint, 5, 0);
	gripperl = 0;
	gripperr = 0;

	ros::Rate rate(10); //Hz
	int readValue;
	static const int numberOfArmJoints = 5;
	static const int numberOfGripperJoints = 2;

	brics_actuator::JointPositions command;
	vector <brics_actuator::JointValue> armJointPositions;
	vector <brics_actuator::JointValue> gripperJointPositions;

	armJointPositions.resize(numberOfArmJoints); //TODO:change that
	gripperJointPositions.resize(numberOfGripperJoints);

	std::stringstream jointName;

	char keyboardInput = '0';

	for(int i = 0; i < numberOfArmJoints; i++)
	{
		jointDelta[i] = (jointMax[i] - jointMin[i]) * 0.02;
	}	

	ros::spinOnce();
	
	for(int i = 0; i < numberOfArmJoints; i++)
	{
		lastJoint[i] = joint[i];
	}
	
	while(n.ok())
	{
		ros::spinOnce();

		keyboardInput = '0';
		readValue = 0;

		cout << "Please provide an arm joint or the gripper to operate on.\n\tType number between 1 and 5, to control joints 1 - 5.\n\tOr hit 6 to go to a preprogrammed position.\n\tOr hit 9 to quit:" << endl;
		cin >> readValue;

		if((readValue >= 1)&&(readValue <= 5))
		{
			jointName.str("");
			jointName << "arm_joint_" << readValue;

			cout << "Selected arm joint " << readValue << endl;
		}

		if(readValue == 6)
		{
			cout << "Select a position:" << endl << "\t1.\tHome" << endl << "\t2.\tCamera View" << endl << "\t3.\tCamera Initialization" << endl << "\t4.\tObject" << endl << "\t5.\tGrasp" << endl << "\t6.\tReturn to joint select" << endl << "\t9.\tExit" << endl;

			int input = 0;
			cin >> input;

			if((input >= 1)&&(input <= 5))
			{	/*
				for(int i = 0; i < numberOfArmJoints; i++)
				{

					jointName.str("");
					jointName << "arm_joint_" << (i + 1);

					armJointPositions[i].joint_uri = jointName.str();
					if(input == 1)
					{
						armJointPositions[i].value = jointHome[i];
						joint[i] = jointHome[i];
					}
					if(input == 2)
					{
						armJointPositions[i].value = jointHome[i];
						joint[i] = jointCamera[i];
					}
					if(input == 3)
					{
						armJointPositions[i].value = jointHome[i];
						joint[i] = jointInitialize[i];
					}
					if(input == 4)
					{
						armJointPositions[i].value = jointHome[i];
						joint[i] = jointObject[i];
					}
					if(input == 5)
					{
						armJointPositions[i].value = jointHome[i];
						joint[i] = jointGrasp[i];
					}

					armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);

					lastJoint[i] = joint[i];
				}*/
				
				if(input == 1)
				{
					for(int i = 0; i < numberOfArmJoints; i++)
					{

						jointName.str("");
						jointName << "arm_joint_" << (i + 1);

						armJointPositions[i].joint_uri = jointName.str();
						armJointPositions[i].value = jointHome[i];

						armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);

						joint[i] = jointHome[i];
						lastJoint[i] = joint[i];
						//cout << jointName << " " << joint[i] << endl;
					}
				}
				if(input == 2)
				{
					for(int i = 0; i < numberOfArmJoints; i++)
					{

						jointName.str("");
						jointName << "arm_joint_" << (i + 1);

						armJointPositions[i].joint_uri = jointName.str();
						armJointPositions[i].value = jointCamera[i];

						armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);

						joint[i] = jointCamera[i];
						lastJoint[i] = joint[i];
						//cout << jointName << " " << joint[i] << endl;
					}
				}
				if(input == 3)
				{
					for(int i = 0; i < numberOfArmJoints; i++)
					{

						jointName.str("");
						jointName << "arm_joint_" << (i + 1);

						armJointPositions[i].joint_uri = jointName.str();
						armJointPositions[i].value = jointInitialize[i];

						armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);

						joint[i] = jointObject[i];
						lastJoint[i] = joint[i];
						//cout << jointName << " " << joint[i] << endl;
					}
				}
				if(input == 4)
				{
					for(int i = 0; i < numberOfArmJoints; i++)
					{

						jointName.str("");
						jointName << "arm_joint_" << (i + 1);

						armJointPositions[i].joint_uri = jointName.str();
						armJointPositions[i].value = jointObject[i];

						armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);

						joint[i] = jointObject[i];
						lastJoint[i] = joint[i];
						//cout << jointName << " " << joint[i] << endl;
					}
				}
				if(input == 5)
				{
					for(int i = 0; i < numberOfArmJoints; i++)
					{

						jointName.str("");
						jointName << "arm_joint_" << (i + 1);

						armJointPositions[i].joint_uri = jointName.str();
						armJointPositions[i].value = jointGrasp[i];

						armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);

						joint[i] = jointGrasp[i];
						lastJoint[i] = joint[i];
						//cout << "joint " << i << " " << joint[i] << endl;
					}
				}
		
				gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
				gripperJointPositions[0].value = 0.01;
				gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);

				gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";
				gripperJointPositions[1].value = 0.01;
				gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);


				cout << "Going to joint command: " << joint[0] << " " <<
						joint[1] << " " <<
						joint[2] << " " <<
						joint[3] << " " <<
						joint[4] << " " << endl;

				command.positions = armJointPositions;
				armPositionsPublisher.publish(command);

				command.positions = gripperJointPositions;
				gripperPositionPublisher.publish(command);

			}

			if(input == 6)
			{
				cout << "Returning..." << endl;
				continue;
			}

			if(input == 9)
			{
				cout << "Exiting..." << endl;
				break;
			}


			continue;
		}

		if(readValue == 9)
		{
			cout << "Exiting..." << endl;
			break;
		}


		cout << "Hit w/s to increase/decrease joint angle." << endl;
		cout << "If you want to stop manipulating this joint, hit 'm' to switch to new joint or exit." << endl;
		cout << "Hit o/l to open/close gripper." << endl;
		cout << "To quit, hit Control + C and then 'm'." << endl;

		keyboardInput = '0';
	
		while((keyboardInput != 'm')&&ros::ok())
		{	
			//ros::spinOnce();
			/*
			if(commandMatch())
			{*/

			cout << "Going to joint command: " << joint[0] << " " <<
					joint[1] << " " <<
					joint[2] << " " <<
					joint[3] << " " <<
					joint[4] << " " << endl;

			keyboardInput = getch();
		
			if (keyboardInput == 'w')
				joint[readValue - 1] += jointDelta[readValue - 1];
			if (keyboardInput == 's') 
				joint[readValue - 1] -= jointDelta[readValue - 1];			
			if(keyboardInput == 'o')
			{
				gripperl += gripperDelta;
				gripperr += gripperDelta;
			}
			if(keyboardInput == 'l')
			{
				gripperl -= gripperDelta;
				gripperr -= gripperDelta;
			}

			if(keyboardInput != '0')
			{

				for(int i = 0; i < numberOfArmJoints; i++)
				{

					jointName.str("");
					jointName << "arm_joint_" << (i + 1);

					if(joint[i] < jointMin[i])
						joint[i] = jointMin[i];
					if(joint[i] > jointMax[i])
						joint[i] = jointMax[i];

					armJointPositions[i].joint_uri = jointName.str();
					armJointPositions[i].value = joint[i];

					armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
				}

				if(gripperl < gripperMin)
					gripperl = gripperMin;
				if(gripperr < gripperMin)
					gripperr = gripperMin;
				if(gripperl > gripperMax)
					gripperl = gripperMax;
				if(gripperr > gripperMax)
					gripperr = gripperMax;

				gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
				gripperJointPositions[0].value = gripperl;
				gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);

				gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";
				gripperJointPositions[1].value = gripperr;
				gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);

				command.positions = armJointPositions;
				armPositionsPublisher.publish(command);

				command.positions = gripperJointPositions;
				gripperPositionPublisher.publish(command);

				for(int i = 0; i <numberOfArmJoints; i++)
				{
					lastJoint[i] = joint[i];
				}

				cout << "Joint position given was: " << joint[readValue - 1] << endl;
			}
		}

		rate.sleep();
	}

	return 0;
}

/* EOF */
