#include "ros/ros.h"
#include "messages/printReceipt.h"

#define TICKETINTERVAL 20

void printCallback(const messages::printReceipt& msg)
{
    ROS_INFO("PRINT REQUESTED");
}

int main(int argc, char **argv)
{
    ros::init(argc,argv, "printListener");
    
    ros::NodeHandle n;
    
    ros::Subscriber sub = n.subscribe("print", 1000, printCallback);
    
    ros::spin();

    return 0;
}
