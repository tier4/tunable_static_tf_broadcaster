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

  void dynamicReconfigureCallback(TfConfig& config, uint32_t level);
};
}  // namespace tunable_static_tf_broadcaster

#endif  // TUNABLE_STATIC_TF_BROADCASTER_H_