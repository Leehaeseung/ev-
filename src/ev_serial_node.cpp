#include "ros/ros.h"
#include "ev_main/CmdMsg.h"

void msgCallback(const ev_main::CmdMsg::ConstPtr &msg)
{
    ROS_INFO("received %d", msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ev_serial_node");
    ros::NodeHandle nh;

    ros::Subscriber cmd_subscriber = nh.subscribe("cmd_msg", 100, msgCallback);

    ros::spin();

    return 0;
}