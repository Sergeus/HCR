/*
 * Copyright (c) 2012 Miguel Sarabia del Castillo
 * Imperial College London
 *
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include "messages/HeadCoordinates.h"
#include "messages/activityStatus.h"
#include "messages/startstop.h"

bool enabled = true;

std::vector<std::string> vectorOfTorsos();

// README: The objective of this part is to obtain the position of the user's
// torso with respect to the base of the robot, and then create a velocity
// command so robot tracks the user.

std::vector<std::string> vectorOfTorsos() 
{
	std::vector<std::string> torsos = std::vector<std::string>();
	torsos.push_back("torso_1");
	torsos.push_back("torso_2");
	torsos.push_back("torso_3");
	torsos.push_back("torso_4");
	torsos.push_back("torso_5");
	torsos.push_back("torso_6");
	torsos.push_back("torso_7");
	torsos.push_back("torso_8");
	torsos.push_back("torso_9");
	torsos.push_back("torso_10");

	return torsos;
}

std::string getHeadFromTorso(std::string torso) {
	std::string head = "head_";
	for (int i=6; i < torso.size(); i++) { // hard coded length of string "torso_"
		head += torso[i];
	}
	return head;
}

bool existsValidtf(std::vector<std::string> originFrame, tf::TransformListener &tfl, std::string destFrame, 
	double timeout) 
{
	//std::cout << "Exists started" << std::endl;
	for (int index = 0; index < originFrame.size(); index++) {
		std::string torso = originFrame[index];
		//std::cout << "Testing torso: " << torso << std::endl;  
		if (tfl.waitForTransform(
			    destFrame,
			    torso,
			    ros::Time::now() - ros::Duration(timeout), //Need recent tf
			    ros::Duration(timeout)
			    )) {
			//std::cout << "Returning true." << std::endl;
			return true;
		}
	}
	//std::cout << "Returning false." << std::endl;
	return false;
}

bool IsCloserTorsoExists(std::string currentTorso, double currentDistance, std::string destFrame,
		tf::TransformListener &tfl, double timeout) {
	std::vector<std::string> torsos = vectorOfTorsos();
	for (int i = 0; i < torsos.size(); i++) {
		std::string comparisonTorso = torsos[i];
		if (comparisonTorso != currentTorso &&
			tfl.waitForTransform(
			destFrame,
			comparisonTorso,
			ros::Time::now() - ros::Duration(timeout),
			ros::Duration(timeout))) {
			geometry_msgs::PointStamped input;
			input.header.frame_id = comparisonTorso;
			input.header.stamp = ros::Time(0);

			input.point.x = 0.0;
			input.point.y = 0.0;
			input.point.z = 0.0;

			geometry_msgs::PointStamped output;

			tfl.transformPoint( destFrame, input, output );


			// Now we use the transformed point to calculate the
			// required translational and rotational speeds.

			// Compute the euclidian distance between the torso and
			// the robot.
			double distance = std::sqrt(
				output.point.x * output.point.x +
				output.point.y * output.point.y
				);
			std::cout << "Comparing new " << comparisonTorso << " at distance " << distance
				<< " to existing " << currentTorso << " at distance " << currentDistance << "." << std::endl;
			if (distance < currentDistance) {
				std::cout << "Found torso " << comparisonTorso << " is closer at distance "
					<< distance << "." << std::endl;
				return true;
			}
		}
	}
	std::cout << "There is no closer torso than " << currentTorso << "." << std::endl;
	return false;
}

bool isValidClosesttf(std::string frame, tf::TransformListener &tfl, std::string destFrame, double timeout) 
{
	bool valid = tfl.waitForTransform(
			destFrame,
			frame,
			ros::Time::now() - ros::Duration(timeout),
			ros::Duration(timeout));
	if (!valid) {
		std::cout << "Torso " << frame << " is not valid anymore." << std::endl;
		return valid;
	}
	else {
		geometry_msgs::PointStamped input;
		input.header.frame_id = frame;
		input.header.stamp = ros::Time(0);

		input.point.x = 0.0;
		input.point.y = 0.0;
		input.point.z = 0.0;

		geometry_msgs::PointStamped output;

		tfl.transformPoint( destFrame, input, output );


		// Now we use the transformed point to calculate the
		// required translational and rotational speeds.

		// Compute the euclidian distance between the torso and
		// the robot.
		double frameDistance = std::sqrt(
			output.point.x * output.point.x +
			output.point.y * output.point.y
			);

		return IsCloserTorsoExists(frame, frameDistance, destFrame, tfl, timeout);
	}
}

std::string findValidtf(std::vector<std::string> originFrame, tf::TransformListener &tfl, std::string destFrame, 
	double timeout) 
{
	std::string foundTorso = "";
	double foundDistance = 10000000;
	//std::cout << "Valid tf started." << std::endl; 
	for (int i = 0; i < originFrame.size(); i++) {
		std::string torso = originFrame[i];
		//std::cout << "Testing torso: " << torso << std::endl;  
		if (tfl.waitForTransform(
			    destFrame,
			    torso,
			    ros::Time::now() - ros::Duration(timeout), //Need recent tf
			    ros::Duration(timeout)
			    )) {

				geometry_msgs::PointStamped input;
				input.header.frame_id = torso;
				input.header.stamp = ros::Time(0);

				input.point.x = 0.0;
				input.point.y = 0.0;
				input.point.z = 0.0;

				geometry_msgs::PointStamped output;

				tfl.transformPoint( destFrame, input, output );


				// Now we use the transformed point to calculate the
				// required translational and rotational speeds.

				// Compute the euclidian distance between the torso and
				// the robot.
				double distance = std::sqrt(
					output.point.x * output.point.x +
					output.point.y * output.point.y
					);

				//std::cout << "Found a likable torso at distance" << distance << "." << std::endl;
				if (distance < foundDistance) {
					foundTorso = torso;
					foundDistance = distance;
				}
		}
	}
	std::cout << "Tracking torso " << foundTorso << " at distance " << foundDistance << "." << std::endl;
	return foundTorso;
} 

void enableRotation(const controlnode::startstop::ConstPtr& msg)
{
    if (msg.operation.compare("START") == 0) {
	enabled = true;
    else
	enabled = false;

}

int main(int argc, char* argv[])
{
    // WARNING: Safety-critical constants, please do not modify.
    const double frequency = 20;
    const double timeToUser = 12; //Time for robot to reach to user's position (default = 6)
    const double minDistance = 0.4; //30 cm
    const double minAngle = 0.09; //~5deg
    const double maxAngle = 0.30; //~45deg?
    const double maxLinearSpeed = 0.3;// in metres per second
    const double maxAngularSpeed = 0.52; //~30 deg per second

    // Initialise ROS
    ros::init(argc, argv, "kinect_follower");
    ros::NodeHandle nodeHandle;
    ros::Rate nodeRate(frequency); //Node runs at 20Hz

    //Advertise cmd_vel
    ros::Publisher pub = nodeHandle.advertise<geometry_msgs::Twist>(
                "cmd_vel", 20
                );

    ros::Publisher headPub = nodeHandle.advertise<messages::HeadCoordinates>(
		"kinect_heads", 20
		);

    ros::Publisher LostPersonPub = nodeHandle.advertise<messages::activityStatus>(
		"kinectLostActivity", 20
		);

    ros::Publisher FoundPersonPub = nodeHandle.advertise<messages::activityStatus>(
		"kinectFoundActivity", 20
		);

    // Maximum time for transform to be available
    const double timeout = 0.05;
    
    // Fill in the values of originFrame and destFrame
    const std::vector<std::string> originFrame = vectorOfTorsos();
    const std::string destFrame = "base_link";

    tf::TransformListener tfl;

    std::string currentTorso = originFrame[0];

    while( ros::ok() )
    {
        double linearSpeed = 0.0;
        double angularSpeed = 0.0;

	bool okay = false;

	if (isValidClosesttf(currentTorso, tfl, destFrame, timeout)) {
		okay = true;
	}
        // Check if transform exists
	else if (existsValidtf(originFrame, tfl, destFrame, timeout)) {
		std::string lostTorso = currentTorso;
		currentTorso = findValidtf(originFrame, tfl, destFrame, timeout);
		if (currentTorso.compare("") != 0) {
			okay = true;
			std::cout << "Tracking new torso " << currentTorso << "." << std::endl;

			messages::activityStatus statusMsg;

			statusMsg.activity = currentTorso;

			FoundPersonPub.publish(statusMsg);
		}
		else {
			messages::activityStatus statusMsg;

			statusMsg.activity = lostTorso;

			LostPersonPub.publish(statusMsg);

			geometry_msgs::Twist msg;

			msg.linear.x = 0;
			msg.angular.z = 0;

			std::cout << "Found no valid tf to track, so not moving." << std::endl;

			//Finally the message gets sent
			pub.publish(msg);

			ros::spinOnce();
			nodeRate.sleep();
		}
	}
	else {
		messages::activityStatus statusMsg;

		statusMsg.activity = currentTorso;

		LostPersonPub.publish(statusMsg);
		
		geometry_msgs::Twist msg;

		msg.linear.x = 0;
		msg.angular.z = 0;

		std::cout << "Found no valid tf to track, so not moving." << std::endl;

		//Finally the message gets sent
		pub.publish(msg);

		ros::spinOnce();
		nodeRate.sleep();
	}

	ros::spinOnce();

	if (okay && enabled) {
		// Here we transform the point (0,0,0) wrt the user's torso
		// to a point wrt to the base of the robot.

		// Set the appropriate values for input
		geometry_msgs::PointStamped input;
		input.header.frame_id = currentTorso;
		input.header.stamp = ros::Time(0);

		input.point.x = 0.0;
		input.point.y = 0.0;
		input.point.z = 0.0;

		geometry_msgs::PointStamped output;

		tfl.transformPoint( destFrame, input, output );

		// Now we use the transformed point to calculate the
		// required translational and rotational speeds.

		// Compute the euclidian distance between the torso and
		// the robot.
		double distance = std::sqrt(
			output.point.x * output.point.x +
			output.point.y * output.point.y
			);

		double angle = std::atan2(output.point.y, output.point.x);

		// The two blocks of code make sure we don't go faster
		// than the limits specified at the beginning of the file.
		if ( std::abs(angle) > minAngle && std::abs(angle) < maxAngle)
		{
			angularSpeed = std::abs( angle*frequency / timeToUser );

			if (angularSpeed > maxAngularSpeed)
			    angularSpeed = maxAngularSpeed;


			if ( angle < 0 )
			    angularSpeed = -angularSpeed;

		}
		else {
			angularSpeed = 0;
		}
	
		if ( distance > minDistance )
		{
			//linearSpeed =  distance * frequency /timeToUser ;
			linearSpeed = 0;

			if (linearSpeed > maxLinearSpeed)
		    		//linearSpeed = maxLinearSpeed;
		    		linearSpeed = 0;

		}

		// Now we need to work out the head coordinates
		std::string currentHead = getHeadFromTorso(currentTorso);

		geometry_msgs::PointStamped headInput;
		headInput.header.frame_id = currentHead;
		headInput.header.stamp = ros::Time(0);

		headInput.point.x = 0.0;
		headInput.point.y = 0.0;
		headInput.point.z = 0.0;

		geometry_msgs::PointStamped headOutput;

		tfl.transformPoint( destFrame, headInput, headOutput );

		kinect_follower::HeadCoordinates headMsg;
		headMsg.x = headOutput.point.y;
		headMsg.y = headOutput.point.z;
		headMsg.z = headOutput.point.x;

		std::cout << "Publishing head coords at x: " << headMsg.x << " y: " << headMsg.y << "." << std::endl;

		headPub.publish(headMsg);

		// Now we have to pack the information in a Twist message.
		// Because the robot only moves in the plane we do not need to fill
		// in all of Twist fields, only linear.x (linear speed) and
		// angular.z (angular speed) are relevant.

		geometry_msgs::Twist msg;

		msg.linear.x = linearSpeed;
		msg.angular.z = angularSpeed;

		std::cout << "Moving by angular speed " << angularSpeed << "." << std::endl;

		//Finally the message gets sent
		pub.publish(msg);

		ros::spinOnce();
		nodeRate.sleep();
	}
    }

    return 0;
}
