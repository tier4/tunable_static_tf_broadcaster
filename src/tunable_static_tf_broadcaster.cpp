#include "tunable_static_tf_broadcaster.h"
#include "tunable_static_tf_broadcaster/TfConfig.h"
#include <tf2/LinearMath/Quaternion.h>

namespace tunable_static_tf_broadcaster
{

TunableStaticTfBroadcaster::TunableStaticTfBroadcaster()
 : private_nh_("~")
{
    // Initialize transform
    transform_.transform.translation.x = 0.0;
    transform_.transform.translation.y = 0.0;
    transform_.transform.translation.z = 0.0;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, 0.0);
    transform_.transform.rotation.x = quaternion.x();
    transform_.transform.rotation.y = quaternion.y();
    transform_.transform.rotation.z = quaternion.z();
    transform_.transform.rotation.w = quaternion.w();

    // Get configuration from param
    publish_rate_ = private_nh_.param<int>("rate", 10);
    transform_.header.frame_id = private_nh_.param<std::string>("header_frame", "world");
    transform_.child_frame_id = private_nh_.param<std::string>("child_frame", "base_link");

    ROS_DEBUG("[tunable_static_tf_broadcaster] Setting parameters... {rate: %d, header_frame: %s, child_frame: %s}",
        publish_rate_, transform_.header.frame_id.c_str(), transform_.child_frame_id.c_str());

    // Setup dynamic reconfigure
    dynamic_reconfigure::Server<TfConfig>::CallbackType f;
    f = boost::bind<void>(&TunableStaticTfBroadcaster::dynamicReconfigureCallback, this, _1, _2);
    dynamic_reconfigure_server_.setCallback(f);
}

void TunableStaticTfBroadcaster::run()
{
    ros::Rate rate(publish_rate_);
    while(private_nh_.ok())
    {
        // Send transform
        transform_.header.stamp = ros::Time::now();
        broadcaster_.sendTransform(transform_);
        ros::spinOnce();
        rate.sleep();
    }
}

void TunableStaticTfBroadcaster::dynamicReconfigureCallback(TfConfig &config, uint32_t level)
{
    transform_.transform.translation.x = config.tf_x;
    transform_.transform.translation.y = config.tf_y;
    transform_.transform.translation.z = config.tf_z;
    tf2::Quaternion quaternion;
    quaternion.setRPY(config.tf_roll, config.tf_pitch, config.tf_yaw);
    transform_.transform.rotation.x = quaternion.x();
    transform_.transform.rotation.y = quaternion.y();
    transform_.transform.rotation.z = quaternion.z();
    transform_.transform.rotation.w = quaternion.w();

    ROS_DEBUG("[tunable_static_tf_broadcaster] Setting dynamic parameters... {x: %lf y: %lf z: %lf roll: %lf pitch: %lf yaw: %lf}",
        config.tf_x, config.tf_y, config.tf_z, config.tf_roll, config.tf_pitch, config.tf_yaw);
}

}