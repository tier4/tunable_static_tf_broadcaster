#include "ros/ros.h"
#include "tunable_static_tf_broadcaster.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tunable_static_tf_broadcaster");
    tunable_static_tf_broadcaster::TunableStaticTfBroadcaster broadcaster;
    broadcaster.run();
}