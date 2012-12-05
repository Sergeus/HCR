#include <iostream>
#include <stdio.h>
#include <time.h>
#include <string>
#include "ros/ros.h"
// #include "kinect_follower/activityStatus.h"
#include "controlnode/activityStatus.h"
#include "controlnode/startstop.h"

#define TICKETINTERVAL 30

using namespace std;

enum behavModesEnum { MODE0, MODE1, MODE2, MODE3 };
behavModesEnum currentBehaviour = MODE2;

enum statesEnum { IDLE, FOLLOWING, SPEAKING, CONVERSING, PRINTING };
statesEnum currentState = IDLE;

void printTicket()
{
    time_t epochTime = time(NULL);
    ROS_INFO("TICKET PRINTED AT TIME %ld", epochTime);   
}

void testFunction(const controlnode::startstop::ConstPtr& msg)
{
    ROS_INFO("TESTFUNCTION CALLED");

}

void publishMessage(ros::Publisher pub, string operation)
{
    controlnode::startstop msg;
    msg.operation = operation;
    pub.publish(msg);
    ROS_INFO("PUBLISHING");
}

int main(int argc, char **argv)
{
    ros::init(argc,argv, "controlnode");
    
    ros::NodeHandle n;
    
    ros::Subscriber sub = n.subscribe("test", 1000, testFunction);
    ros::Publisher kinectSS = n.advertise<controlnode::startstop>("kinectSS", 1000); 
    controlnode::startstop msg;

    ros::Rate loop_rate(1);
    
    // Initialise state
    switch (currentBehaviour)
    {
        case MODE0 :
            currentState = IDLE;
            break;
        case MODE1 :
            currentState = FOLLOWING;
            break;
        case MODE2 :
            currentState = FOLLOWING;
            break;
        case MODE3 :
            currentState = FOLLOWING;
            break;
    }

    int count = 0;

    while (ros::ok())
    {
        ROS_INFO("TEST LOOP (%d)", count);
        count++;
        // cout << currentState; 

        switch (currentBehaviour)
        {
            // This behaviour just prints tickets
            case MODE0 :
                switch (currentState)
                {
                    case IDLE :
                        ROS_INFO("MODE: 0; STATE: IDLE");
                        publishMessage(kinectSS, "STOP");
                        if ((time(NULL) % TICKETINTERVAL) == 0)
                            currentState = PRINTING;
                        break;

                    case PRINTING :
                        ROS_INFO("MODE: 0; STATE: PRINT");
                        printTicket();
                        currentState = IDLE;
                        break;

                    default:
                        currentState = IDLE;
                }
                break;
            
            // This behaviour rotates, and prints tickets
            case MODE1 :
                switch (currentState)
                {
                    case FOLLOWING :
                        ROS_INFO("MODE: 1; STATE: FOLLOWING");
                        publishMessage(kinectSS, "START");
                        break;
                    
                    case PRINTING :
                        ROS_INFO("MODE: 1; STATE: PRINTING");
                        printTicket();
                        currentState = IDLE;
                        break;

                    default:
                        currentState = FOLLOWING;
                }
                break;
            
            // This behaviour rotates, speaks, and prints
            case MODE2 :
                switch (currentState)
                {
                    case FOLLOWING :
                        ROS_INFO("MODE: 2; STATE: FOLLOWING");
                        publishMessage(kinectSS, "START");
                        break;
                    
                    case SPEAKING :
                        ROS_INFO("MODE: 2; STATE: SPEAKING");
                        break;

                    case PRINTING :
                        ROS_INFO("MODE: 2; STATE: PRINTING");
                        printTicket();
                        currentState = IDLE;
                        break;

                    default:
                        currentState = FOLLOWING;
                }
                currentState = FOLLOWING;
                break;

            // This behaviour rotates, converses, and prints
            case MODE3 :
                switch (currentState)
                {
                    case FOLLOWING :
                        ROS_INFO("MODE: 3; STATE: FOLLOWING");
                        publishMessage(kinectSS, "START");
                        break;
                    
                    case CONVERSING :
                        ROS_INFO("MODE: 3; STATE: CONVERSING");
                        break;

                    case PRINTING :
                        ROS_INFO("MODE: 3; STATE: PRINTING");
                        printTicket();
                        currentState = IDLE;
                        break;

                    default:
                        currentState = FOLLOWING;
                }
                currentState = FOLLOWING;
                break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
