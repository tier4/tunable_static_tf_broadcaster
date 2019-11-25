#ifndef TUNABLE_STATIC_TF_BROADCASTER_H_
#define TUNABLE_STATIC_TF_BROADCASTER_H_

#include <memory>
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
        ~TunableStaticTfBroadcaster();
        void run();

    private:
        ros::NodeHandle private_nh_;
        tf2_ros::TransformBroadcaster broadcaster_;
        geometry_msgs::TransformStamped transform_;
        std::shared_ptr<dynamic_reconfigure::Server<TfConfig> > dynamic_reconfigure_server_;
        int publish_rate_;
        bool has_initial_static_params_ = false;
        bool is_first_config_callback_ = true;
        bool is_valid_config_received_ = false;

        void dynamicReconfigureCallback(TfConfig &config, uint32_t level);
    };
}

#endif // TUNABLE_STATIC_TF_BROADCASTER_H_