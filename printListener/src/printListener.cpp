#include <time.h>
#include <sstream>
#include <string>
#include "ros/ros.h"
#include "messages/printReceipt.h"

#define TICKETINTERVAL 20
#define PRINTERUSB 2

std::string messagePath = "/home/human/message.txt";

void printCallback(const messages::printReceipt& msg)
{
    ROS_INFO("PRINT REQUESTED");

    static time_t lastPrint = time(NULL);
    time_t epochTime = time(NULL);

    std::stringstream command; 
    // command << "echo 'MODE" << msg.mode << " at time " << epochTime 
    command << "echo 'MODE" << " at time " << epochTime 
            << "' >> /home/human/charleslog; /home/human/ros_workspace/printer/c++/async " 
            << PRINTERUSB << " " << epochTime << " " << messagePath << " true 0 true";
    
    if ((int)difftime(epochTime,lastPrint) >= TICKETINTERVAL)
    {
        // std::cout << "STR: "<< command.str() << std::endl;
        system(command.str().c_str());
        lastPrint = epochTime;
        ROS_INFO("TICKET PRINTED AT TIME %ld", epochTime);   
    } else {
        ROS_INFO("TOO RECENT, NO PRINT"); 
    }
}

int main(int argc, char **argv)
{
    ros::init(argc,argv, "printListener");
    
    ros::NodeHandle n;
    
    ros::Subscriber sub = n.subscribe("print", 1000, printCallback);
    
    ros::spin();

    return 0;
}
