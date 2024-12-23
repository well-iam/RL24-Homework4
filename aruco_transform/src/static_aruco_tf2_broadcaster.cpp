// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <iostream>
#include <memory>
#include <cstdlib>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std;

class StaticFramePublisher : public rclcpp::Node
{
public:
  explicit StaticFramePublisher() 
  : Node("static_aruco_tf2_broadcaster")
  {
    aruco_pose_available_ = false; 
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    ArucoPos_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/aruco_single/pose", 10, std::bind(&StaticFramePublisher::aruco_pose, this, std::placeholders::_1));
    //  Waiting for the /aruco_single/pose topic
  RCLCPP_INFO(this->get_logger(), "Aruco Pose not available ...");  
  }

private:
  void make_transforms()
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "aruco_marker_frame";

    t.transform.translation.x = vec[0]; //double
    t.transform.translation.y = vec[1];
    t.transform.translation.z = vec[2];

    t.transform.rotation.x = quat[0];
    t.transform.rotation.y = quat[1];
    t.transform.rotation.z = quat[2];
    t.transform.rotation.w = quat[3];

    tf_static_broadcaster_->sendTransform(t);
  }

  void aruco_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    //position
    vec[0] = msg->pose.position.x;
    vec[1] = msg->pose.position.y;
    vec[2] = msg->pose.position.z;
    //orientation
    quat[0] =msg->pose.orientation.x;
    quat[1] =msg->pose.orientation.y;
    quat[2] =msg->pose.orientation.z;
    quat[3] =msg->pose.orientation.w;
    this->make_transforms();
    aruco_pose_available_=true;
    std::cout <<"Aruco Pose Published as TF!" << std::endl; 
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ArucoPos_;
  bool aruco_pose_available_;
  double vec[3];
  double quat[4];

};

int main(int argc, char * argv[])
{
  // Pass parameters and initialize node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticFramePublisher>(/*argv*/));
  rclcpp::shutdown();
  return 0;
}
