/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include "tunable_static_tf_broadcaster/tunable_static_tf_broadcaster.h"
#include "tunable_static_tf_broadcaster/TfConfig.h"


namespace tunable_static_tf_broadcaster
{
TunableStaticTfBroadcaster::TunableStaticTfBroadcaster() : private_nh_("~")
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

  // Get if static param is set
  if (private_nh_.hasParam("tf_x") || private_nh_.hasParam("tf_y") || private_nh_.hasParam("tf_z") ||
      private_nh_.hasParam("tf_roll") || private_nh_.hasParam("tf_pitch") || private_nh_.hasParam("tf_yaw"))
  {
    has_initial_static_params_ = true;
  }
  else
  {
    has_initial_static_params_ = false;
  }

  // Setup dynamic reconfigure
  dynamic_reconfigure_server_ = std::make_shared<dynamic_reconfigure::Server<TfConfig> >();
  dynamic_reconfigure::Server<TfConfig>::CallbackType f;
  f = boost::bind(&TunableStaticTfBroadcaster::dynamicReconfigureCallback, this, _1, _2);
  dynamic_reconfigure_server_->setCallback(f);
}

TunableStaticTfBroadcaster::~TunableStaticTfBroadcaster()
{
  // Delete params for another run
  if (!is_valid_config_received_)
  {
    private_nh_.deleteParam("tf_x");
    private_nh_.deleteParam("tf_y");
    private_nh_.deleteParam("tf_z");
    private_nh_.deleteParam("tf_roll");
    private_nh_.deleteParam("tf_pitch");
    private_nh_.deleteParam("tf_yaw");
  }
}

void TunableStaticTfBroadcaster::run()
{
  ros::Rate rate(publish_rate_);
  ros::Duration duration(1.0 / publish_rate_);

  while (private_nh_.ok())
  {
    // Send transform
    if (is_valid_config_received_)
    {
      transform_.header.stamp = ros::Time::now() + duration;  // Set future time to allow
                                                              // slower sending w/o other node's timeout.
      broadcaster_.sendTransform(transform_);
    }
    else
    {
      ROS_WARN_THROTTLE(3.0, "[tunable_static_tf_broadcaster] Configuration not received. waiting...");
    }

    ros::spinOnce();
    rate.sleep();
  }
}

void TunableStaticTfBroadcaster::dynamicReconfigureCallback(TfConfig& config, uint32_t level)
{
  if (is_first_config_callback_)
  {
    // If it was first calling, there are cases that it has static param value or not.
    if (has_initial_static_params_)
    {
      // Get static param value
      transform_.transform.translation.x = config.tf_x;
      transform_.transform.translation.y = config.tf_y;
      transform_.transform.translation.z = config.tf_z;
      tf2::Quaternion quaternion;
      quaternion.setRPY(config.tf_roll, config.tf_pitch, config.tf_yaw);
      transform_.transform.rotation.x = quaternion.x();
      transform_.transform.rotation.y = quaternion.y();
      transform_.transform.rotation.z = quaternion.z();
      transform_.transform.rotation.w = quaternion.w();
      is_valid_config_received_ = true;

      ROS_DEBUG("[tunable_static_tf_broadcaster] Setting parameters by static params... {x: %lf y: %lf z: %lf roll: "
                "%lf pitch: %lf yaw: %lf}",
                config.tf_x, config.tf_y, config.tf_z, config.tf_roll, config.tf_pitch, config.tf_yaw);
    }
    else
    {
      // Static param is not set. Wait for dynamic configuration.
      is_valid_config_received_ = false;
    }
    is_first_config_callback_ = false;
  }
  else
  {
    // Not first calling
    transform_.transform.translation.x = config.tf_x;
    transform_.transform.translation.y = config.tf_y;
    transform_.transform.translation.z = config.tf_z;
    tf2::Quaternion quaternion;
    quaternion.setRPY(config.tf_roll, config.tf_pitch, config.tf_yaw);
    transform_.transform.rotation.x = quaternion.x();
    transform_.transform.rotation.y = quaternion.y();
    transform_.transform.rotation.z = quaternion.z();
    transform_.transform.rotation.w = quaternion.w();
    is_valid_config_received_ = true;

    ROS_DEBUG("[tunable_static_tf_broadcaster] Setting dynamic parameters... {x: %lf y: %lf z: %lf roll: %lf pitch: "
              "%lf yaw: %lf}",
              config.tf_x, config.tf_y, config.tf_z, config.tf_roll, config.tf_pitch, config.tf_yaw);
  }
}

}  // namespace tunable_static_tf_broadcaster
