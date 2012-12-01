#include "flite.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

cst_voice *register_cmu_us_kal();
cst_voice *v;

void callback(const std_msgs::String::ConstPtr& msg){

  flite_text_to_speech(msg->data.c_str(),v,"play");

  ROS_INFO("I heard: [%s]", msg->data.c_str());

}

int main(int argc, char **argv){

  ros::init(argc, argv, "ros_flite");

  flite_init();
  v = register_cmu_us_kal();

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("TTS", 1000, callback);

  ros::spin();

  return 0;
}
