#include "removert/Removerter.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "removert");
    ROS_INFO("\033[1;32m----> Removert Main Started.\033[0m");

    Removerter RMV;
    RMV.run();

    ros::spin();

    return 0;
}