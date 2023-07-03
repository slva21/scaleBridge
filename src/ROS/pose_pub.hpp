#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>

typedef Eigen::Matrix<float, 9, 1> StateType; // X = [x, y, z, roll, pitch, yaw, vx, vy, vz]^T

class PosePublisher
{
public:
  PosePublisher(std::string topic_name)
  {
    ros::NodeHandle nh;
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(topic_name, 10);
  }

  void publishPose(const StateType &state)
  {
    geometry_msgs::PoseStamped pose_msg;

    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "world"; // Set the frame to "world"
    pose_msg.pose.position.x = state[0];
    pose_msg.pose.position.y = state[1];
    pose_msg.pose.position.z = state[2];

    // Convert roll, pitch, and yaw to quaternion
    tf2::Quaternion quat;
    quat.setRPY(state[3], state[4], state[5]);
    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();
    pose_msg.pose.orientation.w = quat.w();

    pose_pub_.publish(pose_msg);
  }

  void publishPose(const Eigen::Isometry3d &state)
  {
    geometry_msgs::PoseStamped pose_msg;

    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "world"; // Set the frame to "world"
    pose_msg.pose.position.x = state.translation()[0];
    pose_msg.pose.position.y = state.translation()[1];
    pose_msg.pose.position.z = state.translation()[2];

    Eigen::Quaterniond quat(state.rotation());
    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();
    pose_msg.pose.orientation.w = quat.w();

    pose_pub_.publish(pose_msg);
  }

private:
  ros::Publisher pose_pub_;
};
