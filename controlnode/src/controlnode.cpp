#include <ros/ros.h> 

enum behavModesEnum { MODE1, MODE2, MODE3 };
behavModesEnum currentBehaviour = MODE1;

enum statesEnum { IDLE, FOLLOWING, CONVERSING, PRINTING };
statesEnum currentState = IDLE;

bool stateChanged = true;

int main(int argc, char **argv)
{
    ros::init(argc,argv, "controlnode");
    
    ros::NodeHandle n;
    
    // ros:Rate loop_rate(10);

    while (ros::ok())
    {
        ROS_INFO("TEST LOOP");

        switch ( currentState )
        {
            case IDLE :
                break;
            case FOLLOWING :
                break;
            case CONVERSING :
                break;
            case PRINTING :
                break;
        }

        // If the state has changed send necessary messages
        if ( stateChanged ) {

            stateChanged = false;
        }
    }

    return 0;
}
