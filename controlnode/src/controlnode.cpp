#include <iostream>
#include <stdio.h>
#include <time.h>
#include <string>
#include <set>
#include "ros/ros.h"
#include "messages/activityStatus.h"
#include "messages/startstop.h"

#define TICKETINTERVAL 30

enum behavModesEnum { MODE0, MODE1, MODE2, MODE3 };
behavModesEnum currentBehaviour = MODE0;

enum statesEnum { IDLE, FOLLOWING, SPEAKING, CONVERSING, PRINTING };
statesEnum currentState = IDLE;

std::set<std::string> torsos;

void printTicket()
{
    time_t epochTime = time(NULL);
    ROS_INFO("TICKET PRINTED AT TIME %ld", epochTime);   
}

bool participantPresent()
{
    ROS_INFO("NUMBER of people is %d", torsos.size());
    if (torsos.size() > 0)
        return true;
    else
        return false;
}

void kinectLostCallback(const messages::activityStatus& msg)
{
    torsos.erase(msg.activity);
}

void kinectFoundCallback(const messages::activityStatus& msg)
{
    torsos.insert(msg.activity);
}

void publishMessage(ros::Publisher pub, std::string operation)
{
    messages::startstop msg;
    msg.operation = operation;
    pub.publish(msg);
    // ROS_INFO("PUBLISHING");
}

int main(int argc, char **argv)
{
    ros::init(argc,argv, "controlnode");
    
    ros::NodeHandle n;
    
    ros::Subscriber kinectLostSub = n.subscribe("kinectLostActivity", 1000, kinectLostCallback);
    ros::Subscriber kinectFoundSub = n.subscribe("kinectFoundActivity", 1000, kinectFoundCallback);
    
    ros::Publisher kinectSS = n.advertise<messages::startstop>("kinectSS", 1000); 
    ros::Publisher speakSS = n.advertise<messages::startstop>("speakSS", 1000); 
    ros::Publisher converseSS = n.advertise<messages::startstop>("converseSS", 1000); 

    messages::startstop msg;

    // ros::Rate loop_rate(0.2);
    ros::Rate loop_rate(1);
    
    // Set mode from command line
    switch (atoi(argv[1]))
    {
        case 0:
            currentBehaviour = MODE0;
            break;
        case 1:
            currentBehaviour = MODE1;
            break;
        case 2:
            currentBehaviour = MODE2;
            break;
        case 3:
            currentBehaviour = MODE3;
            break;
        default :
            currentBehaviour = MODE0;
    }

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
        ROS_INFO("WHILELOOP (%d)", count++);

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
                        if (participantPresent())
                            currentState = PRINTING;
                        break;
                    
                    case PRINTING :
                        ROS_INFO("MODE: 1; STATE: PRINTING");
                        printTicket();
                        currentState = FOLLOWING;
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
                        if (!participantPresent())
                            currentState = IDLE;
                        break;
                    
                    case SPEAKING :
                        ROS_INFO("MODE: 2; STATE: SPEAKING");
                        if (!participantPresent())
                            currentState = FOLLOWING;
                        break;

                    case PRINTING :
                        ROS_INFO("MODE: 2; STATE: PRINTING");
                        printTicket();
                        if (participantPresent())
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
