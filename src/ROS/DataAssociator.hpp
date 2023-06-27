#pragma once

#include <ros/ros.h>
#include <iostream>

#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <mutex>

class DataAssociator
{
public:
    // constructor
    DataAssociator() {}

    void addImuData(const Eigen::VectorXd &data_imu, const ros::Time &time_imu)
    {
        std::lock_guard<std::mutex> lock(buffer_mutex);
        imu_buffer.push_back(std::make_pair(data_imu, time_imu));

        // Remove old data if buffer is too large
        if (imu_buffer.size() > buffer_size)
        {
            imu_buffer.pop_front();
        }
    }

    void addLsdData(const Eigen::VectorXd &data_lsd, const ros::Time &time_lsd)
    {
        std::lock_guard<std::mutex> lock(buffer_mutex);
        lsd_buffer.push_back(std::make_pair(data_lsd, time_lsd));

        // Remove old data if buffer is too large
        if (lsd_buffer.size() > buffer_size)
        {
            lsd_buffer.pop_front();
        }
    }

    bool getMatchingData(std::pair<Eigen::VectorXd, Eigen::VectorXd> &matching_data)
    {
        std::lock_guard<std::mutex> lock(buffer_mutex);

        // iterate over all the elements in the lsd_buffer
        for (auto &lsd_data_time : lsd_buffer)
        {
            // iterate over all the elements in the imu_buffer
            for (auto &imu_data_time : imu_buffer)
            {
                // Check if the timestamps are similar within a certain threshold
                ros::Duration time_diff = imu_data_time.second - lsd_data_time.second;
                if (std::abs(time_diff.toSec()) < time_threshold)
                {
                    // If the timestamps are close enough, assign the matching pair
                    matching_data = std::make_pair(imu_data_time.first, lsd_data_time.first);
                    return true;
                }
            }
        }

        // If no matching data found, return false
        return false;
    }

private:
    std::deque<std::pair<Eigen::VectorXd, ros::Time>> imu_buffer;
    std::deque<std::pair<Eigen::VectorXd, ros::Time>> lsd_buffer;
    std::mutex buffer_mutex;
    const size_t buffer_size = 100;     // replace with your desired buffer size
    const double time_threshold = 0.01; // time difference threshold in seconds for data matching
};
