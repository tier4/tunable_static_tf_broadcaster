#ifndef TUNABLE_STATIC_TF_BROADCASTER_H_
#define TUNABLE_STATIC_TF_BROADCASTER_H_

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <dynamic_reconfigure/server.h>
#include "tunable_static_tf_broadcaster/TfConfig.h"

namespace tunable_static_tf_broadcaster
{
    class TunableStaticTfBroadcaster
    {
    public:
        TunableStaticTfBroadcaster();
        void run();

    private:
        ros::NodeHandle private_nh_;
        tf2_ros::TransformBroadcaster broadcaster_;
        geometry_msgs::TransformStamped transform_;
        dynamic_reconfigure::Server<TfConfig> dynamic_reconfigure_server_;
        int publish_rate_;

        void dynamicReconfigureCallback(TfConfig &config, uint32_t level);
    };
}

#endif // TUNABLE_STATIC_TF_BROADCASTER_H_