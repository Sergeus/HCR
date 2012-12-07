#include <iostream>
#include <sstream>
#include <stdio.h>
#include <time.h>
#include <string>
#include <set>
#include "ros/ros.h"
#include "messages/activityStatus.h"
#include "messages/startstop.h"
#include "messages/printRequest.h"
#include "messages/conversationFinished.h"

#define REFRESHFREQ 1
#define TICKETINTERVAL 20
#define PRINTERUSB 2

std::string messagePath = "/home/human/message.txt";

enum behavModesEnum { MODE0, MODE1, MODE2, MODE3 };
behavModesEnum currentBehaviour = MODE0;

enum statesEnum { IDLE, FOLLOWING, SPEAKING, CONVERSING, PRINTING };
statesEnum currentState = IDLE;

std::set<std::string> torsos;
bool printRequested = false;
time_t lastConversation = time(NULL);
bool inConversatoin = false;

void printTicket()
{
    static time_t lastPrint = time(NULL);
    time_t epochTime = time(NULL);

    std::stringstream command; 
    command << "echo 'MODE" << currentBehaviour << " at time " << epochTime 
            << "' >> /home/human/charleslog; /home/human/ros_workspace/printer/c++/async " 
            << PRINTERUSB << " " << epochTime << " " << messagePath << " true 0 true";
    
    if ((int)difftime(epochTime,lastPrint) >= TICKETINTERVAL)
    {
        //std::cout << "STR: "<< command.str() << std::endl;
        system(command.str().c_str());
	lastPrint = epochTime;
        ROS_INFO("TICKET PRINTED AT TIME %ld", epochTime);   
    } else {
        ROS_INFO("TOO RECENT, NO PRINT"); 
    }

    printRequested = false;
}

bool participantPresent()
{
    ROS_INFO("NUMBER of people is %d", torsos.size());
    // std::set<std::string>::iterator it;
    // std::cout << "torsos present: ";
    // for(it=torsos.begin(); it!=torsos.end(); it++)
    // {
    //     std::cout << *it;
    // }
    // std::cout << std::endl;
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

void printRequestCallback(const messages::printRequest& msg)
{
    printRequested = true;
}

void endConversationCallback(const messages::conversationFinished& msg)
{
    lastConversation = time(NULL);
    inConversation = false;
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
    ros::Subscriber printRequestSub = n.subscribe("voicePrintRequests", 1000, printRequestCallback);
    ros::Subscriber endConversationSub = n.subscribe("conversationFinished", 1000, endConversationCallback);

    ros::Publisher kinectSS = n.advertise<messages::startstop>("kinectSS", 1000); 
    ros::Publisher voice_recogSS = n.advertise<messages::startstop>("voice_recogSS", 1000); 

    messages::startstop msg;

    ros::Rate loop_rate(REFRESHFREQ);
    
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

    printTicket(); // This sets the lastPrint time but will not print
    time_t lastSpeech = time(NULL);
    int count = 0;

    while (ros::ok())
    {
        // ROS_INFO("WHILELOOP (%d)", count++);

        ros::spinOnce();
        
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
                        ROS_INFO("UNEXPECTED STATE, RESETTING");
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
                        ROS_INFO("UNEXPECTED STATE, RESETTING");
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
                        publishMessage(voice_recogSS, "STOP");
                        
                        if (participantPresent())
                            currentState = SPEAKING;
                        
                        if (printRequested)
                            currentState = PRINTING;
                        break;
                    
                    case SPEAKING :
                        ROS_INFO("MODE: 2; STATE: SPEAKING");
                        
                        if ((int)difftime(time(NULL), lastSpeech) >= (TICKETINTERVAL +2))
                        {
                            lastSpeech = time(NULL);
                            publishMessage(voice_recogSS, "STARTSPEAKING");
                        } else
                            ROS_INFO("SPOKE RECENTLY, WAITING");
                        
                        if (!participantPresent())
                            currentState = FOLLOWING;
                        
                        if (printRequested)
                            currentState = PRINTING;
                        break;

                    case PRINTING :
                        ROS_INFO("MODE: 2; STATE: PRINTING");
                        printTicket();
                        currentState = FOLLOWING;
                        break;

                    default:
                        ROS_INFO("UNEXPECTED STATE, RESETTING");
                        currentState = FOLLOWING;
                }
                break;

            // This behaviour rotates, converses, and prints
            case MODE3 :
                switch (currentState)
                {
                    case FOLLOWING :
                        ROS_INFO("MODE: 3; STATE: FOLLOWING");
                        
                        publishMessage(kinectSS, "START");
                        publishMessage(voice_recogSS, "STOP");
                        
                        if (participantPresent())
                            currentState = CONVERSING;
                        
                        if (printRequested)
                            currentState = PRINTING;
                        break;
                    
                    case CONVERSING :
                        ROS_INFO("MODE: 3; STATE: CONVERSING");
                        
                        if (((int)difftime(time(NULL),lastConversation)>=TICKETINTERVAL) && !inConversation)
                        {
                            publishMessage(voice_recogSS, "STARTCONVERSING");
                            lastConversation = time(NULL);
                            inConversation = true;
                        } else
                            ROS_INFO("LAST CONVERSATION WAS TOO RECENT");

                        if (!participantPresent())
                            currentState = FOLLOWING;
                        
                        if (printRequested)
                            currentState = PRINTING;
                        break;

                    case PRINTING :
                        ROS_INFO("MODE: 3; STATE: PRINTING");
                        printTicket();
                        currentState = FOLLOWING;
                        break;

                    default:
                        ROS_INFO("UNEXPECTED STATE, RESETTING");
                        currentState = FOLLOWING;
                }
                break;
        }

        loop_rate.sleep();
    }

    return 0;
}
