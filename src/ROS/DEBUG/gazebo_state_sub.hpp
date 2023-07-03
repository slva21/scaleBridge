#pragma once

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <Eigen/Dense>
#include <mutex>

class GazeboModelListener
{
public:
    GazeboModelListener()
    {
        ros::NodeHandle n;
        model_states_sub = n.subscribe("/gazebo/model_states", 10, &GazeboModelListener::modelStatesCallback, this);
    }

    Eigen::Vector3d getLinearVel()
    {
        std::lock_guard<std::mutex> lock(lock_mutex);
        return linear_vel;
    }

    Eigen::Vector3d getAngularVel()
    {
        std::lock_guard<std::mutex> lock(lock_mutex);
        return angular_vel;
    }

    Eigen::Vector3d getSafePosition()
    {
        std::lock_guard<std::mutex> lock(lock_mutex);
        return position;
    }

    Eigen::Vector4d getSafeOrientation()
    {
        std::lock_guard<std::mutex> lock(lock_mutex);
        return orientation;
    }

private:
    ros::Subscriber model_states_sub;
    Eigen::Vector3d position;
    Eigen::Vector4d orientation;
    Eigen::Vector3d linear_vel;
    Eigen::Vector3d angular_vel;
    std::mutex lock_mutex;

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == "sjtu_drone")
            {
                std::lock_guard<std::mutex> lock(lock_mutex);
                orientation = Eigen::Vector4d(msg->pose[i].orientation.x,
                                              msg->pose[i].orientation.y,
                                              msg->pose[i].orientation.z,
                                              msg->pose[i].orientation.w);
                linear_vel = Eigen::Vector3d(msg->twist[i].linear.x,
                                             msg->twist[i].linear.y,
                                             msg->twist[i].linear.z);
                angular_vel = Eigen::Vector3d(msg->twist[i].angular.x,
                                              msg->twist[i].angular.y,
                                              msg->twist[i].angular.z);

                position = Eigen::Vector3d(msg->pose[i].position.x,
                                           msg->pose[i].position.y,
                                           msg->pose[i].position.z);

                break;
            }
        }
    }
};
