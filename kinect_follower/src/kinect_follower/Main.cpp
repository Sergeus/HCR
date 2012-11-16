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



// README: The objective of this part is to obtain the position of the user's
// torso with respect to the base of the robot, and then create a velocity
// command so robot follows the user.

int main(int argc, char* argv[])
{
    // WARNING: Safety-critical constants, please do not modify.
    const double frequency = 20;
    const double timeToUser = 12; //Time for robot to reach to user's position (default = 6)
    const double minDistance = 0.4; //30 cm
    const double minAngle = 0.09; //~5deg
    const double maxLinearSpeed = 0.3;// in metres per second
    const double maxAngularSpeed = 0.52; //~30 deg per second (default = 0.52)

    // Initialise ROS
    ros::init(argc, argv, "kinect_follower");
    ros::NodeHandle nodeHandle;
    ros::Rate nodeRate(frequency); //Node runs at 20Hz

    //Advertise cmd_vel
    ros::Publisher pub = nodeHandle.advertise<geometry_msgs::Twist>(
                "cmd_vel", 10
                );

    // Maximum time for transform to be available
    const double timeout = 0.1;
    // TODO: Fill in the values of originFrame and destFrame
    const std::string originFrame = "torso_1";
    const std::string destFrame = "base_link";


    tf::TransformListener tfl;

    while( ros::ok() )
    {
        // WARNING:The default values are safety-critical, please do not modify.
        double linearSpeed = 0.0;
        double angularSpeed = 0.0;

        // TODO: Does this function seem familiar?
        // Do you understand what it does?
        bool okay = tfl.waitForTransform(
                    destFrame,
                    originFrame,
                    ros::Time::now() - ros::Duration(timeout), //Need recent tf
                    ros::Duration(timeout)
                    );

        if ( okay )
        {
            // README: Here we transform the point (0,0,0) wrt the user's torso
            // to a point wrt to the base of the robot.
            // NOTE: We could also use the transform between the two frames
            // (as in tf_tutorial/step1), but then we would have to explicitly
            // take care of rotations.
            // HINT: Review the solution of tf_tutorial/step3 for inspiration

            // TODO: Set the appropriate values for input
            geometry_msgs::PointStamped input;
            input.header.frame_id = originFrame;
            input.header.stamp = ros::Time(0);

            input.point.x = 0.0;
            input.point.y = 0.0;
            input.point.z = 0.0;

            geometry_msgs::PointStamped output;

            // TODO: Do you understand what this function is doing?
            tfl.transformPoint( destFrame, input, output );


            // README: Now we use the transformed point to calculate the
            // required translational and rotational speeds.

            // TODO: Compute the euclidian distance between the torso and
            // the robot.
            // HINT: To make things simpler, we project the point to the plane
            // of the robot (in other words, we ignore the z value)
            double distance = std::sqrt(
                        output.point.x * output.point.x +
                        output.point.y * output.point.y
                        );

            // TODO: Do you understand what this line of code is doing?
            // HINT: http://www.cplusplus.com/reference/clibrary/cmath/atan2/
            double angle = std::atan2(output.point.y, output.point.x);

            // README: The two blocks of code make sure we don't go faster
            // than the limits specified at the beginning of the file.
            // WARNING: These blocks are safety-critical, please do not modify.
            if ( std::abs(angle) > minAngle )
            {
                angularSpeed = std::abs( angle*frequency / timeToUser );

                if (angularSpeed > maxAngularSpeed)
                    angularSpeed = maxAngularSpeed;


                if ( angle < 0 )
                    angularSpeed = -angularSpeed;

            }
            if ( distance > minDistance )
            {
               // linearSpeed =  distance * frequency /timeToUser ;
		linearSpeed = 0;

                if (linearSpeed > maxLinearSpeed)
                    //linearSpeed = maxLinearSpeed;
		    linearSpeed = 0;

            }
        }

        // README: Now we have to pack the information in a Twist message.
        // Because the robot only moves in the plane we do not need to fill
        // in all of Twist fields, only linear.x (linear speed) and
        // angular.z (angular speed) are relevant.

        geometry_msgs::Twist msg;

        // TODO: Fill in with the correct values
        msg.linear.x = linearSpeed;
        msg.angular.z = angularSpeed;

        //Finally the message gets sent
        pub.publish(msg);

        ros::spinOnce();
        nodeRate.sleep();
    }

    return 0;
}
