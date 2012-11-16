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

#include <cmath>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char* argv[])
{
    // Initialise ROS
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle nodeHandle;
    (void)nodeHandle;
    ros::Rate nodeRate(10); //Node runs at 10Hz

    tf::TransformBroadcaster tfb;

    while( ros::ok() )
    {
        // TODO: Measure the where is the kinect situated with respect to the
        // robot. Remember x = forwards, y = left, z= up.
        // Then write a transform broadcaster from "openni_depth_frame" to
        // "base_link". The values of the transform are the ones you just
        // measured.
        // HINT: Use the solution of tf_tutorial/step2 to fill in the code.
        // HINT: Remember ROS uses metres as distance unit.

        tf::StampedTransform transform;

        // Kinect offset (will change depending on your setup)
        transform.setOrigin( tf::Vector3(0.25, 0.0, 0.18) );

        // No rotation in this case
        tf::Quaternion q;
        q.setEuler(0.0, 0.0, 0.0);
        transform.setRotation( q );

        //Current timestamp
        transform.stamp_ = ros::Time::now();

        //Set parent and children
        transform.frame_id_ = "base_link";
        transform.child_frame_id_ = "openni_depth_frame";

        //Send the transform awaya
        tfb.sendTransform( transform );

        ros::spinOnce();
        nodeRate.sleep();
    }

    return 0;
}
