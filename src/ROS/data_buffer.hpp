#pragma once 

/**
 * @file imu_buffer.hpp
 * 
 * This file contains the declaration of the ImuBuffer class. 
 * The ImuBuffer class is designed to hold a buffer of IMU data as pairs of 
 * Eigen VectorXd objects and corresponding timestamps, stored in a deque data structure.
 *
 * The class provides two main functionalities:
 *   - addPair(): This function is used to add a new pair of Eigen VectorXd and timestamp to the buffer.
 *   - fetchPair(): This function fetches the least recently added pair from the buffer and removes it. 
 *                  It also calculates and returns the time difference between the fetched pair and the pair 
 *                  previously fetched from the buffer. If the buffer is empty, the function returns false.
 *
 * The ImuBuffer classas the IMU data is constantly being received 
 * and processed by the ekf and the order and timing of the received data is important.
 */

#include <deque>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <utility>
#include <mutex>

class DataBuffer {
private:
    std::deque<std::pair<Eigen::VectorXd, ros::Time>> data_buffer;
    ros::Time last_fetch_time;
    std::mutex mtx;
    const size_t max_buffer_size = 20;

public:
    DataBuffer() {}

    void addPair(const Eigen::VectorXd& data, const ros::Time& time) {
        std::lock_guard<std::mutex> lock(mtx);
        if (data_buffer.size() >= max_buffer_size) {
            data_buffer.pop_front();
        }
        data_buffer.push_back(std::make_pair(data, time));
    }

    bool fetchData(Eigen::VectorXd& data, double& dt) {
        std::lock_guard<std::mutex> lock(mtx);
        if (data_buffer.empty()) {
            return false;
        } else {
            auto pair = data_buffer.front();
            data_buffer.pop_front();
            data = pair.first;
            if (!last_fetch_time.isZero()) {
                dt = (pair.second - last_fetch_time).toSec();
            }
            last_fetch_time = pair.second;
            return true;
        }
    }
};

