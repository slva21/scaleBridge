#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Core>

typedef Eigen::Matrix<float, 3, 1> AccelerometerType; // Accelerometer measurements: [ax, ay, az]^T

class AccelerometerPublisher
{
public:
    AccelerometerPublisher()
    {
        ros::NodeHandle nh;
        imu_pub_ = nh.advertise<sensor_msgs::Imu>("visual_intertial/debug/imu", 10);
    }

    void publishAccelerometer(const AccelerometerType &accelerometer)
    {
        sensor_msgs::Imu imu_msg;

        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "world";
        imu_msg.linear_acceleration.x = accelerometer[0];
        imu_msg.linear_acceleration.y = accelerometer[1];
        imu_msg.linear_acceleration.z = accelerometer[2];

        imu_pub_.publish(imu_msg);
    }

private:
    ros::Publisher imu_pub_;
};

// int main(int argc, char** argv) {
//   ros::init(argc, argv, "accelerometer_publisher_node");
//   AccelerometerPublisher accelerometer_publisher;

//   // Example usage:
//   AccelerometerType accelerometer;
//   accelerometer << 1.0, 2.0, 3.0;
//   accelerometer_publisher.publishAccelerometer(accelerometer);

//   ros::spin();
//   return 0;
// }
