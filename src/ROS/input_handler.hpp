#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <Eigen/Geometry>

#include <mutex>
#include "visual_inertial/single_kalman.hpp"
#include "data_buffer.hpp"

#include "./DataAssociator.hpp"
#include "./pose_pub.hpp"
#include "./DEBUG/acc_pub.hpp"

#include "./DEBUG/gazebo_state_sub.hpp"

// rosrun lsd_slam_core live_slam image:=/camera/image_raw camera_info:=/kinect/depth/camera_info

typedef Eigen::Matrix<float, 9, 1> State;

class INPUT_HANDLER
{
public:
    INPUT_HANDLER(State &state) : dt(0.005),
                                  A(Eigen::MatrixXd::Zero(6, 6)),
                                  C(Eigen::MatrixXd::Identity(6, 6)),
                                  Q(Eigen::MatrixXd::Identity(6, 6)),
                                  R(Eigen::MatrixXd::Identity(6, 6)),
                                  P(Eigen::MatrixXd::Identity(6, 6)),
                                  efk_state(state), vio_pose_pub("visual_intertial/vio")
    {
        // Define the state transition matrix A
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
        A << I, dt * I,
            Eigen::MatrixXd::Zero(3, 3), I;

        // Define the Kalman filter
        kf = KalmanFilter(dt, A, C, Q, R, P);

        // Initialize NodeHandle
        ros::NodeHandle nh;

        // Initialize subscribers and publishers
        lsd_pose_sub = nh.subscribe("/lsd_slam/pose", 10, &INPUT_HANDLER::lsdPoseCallback, this);
        imu_data_sub = nh.subscribe("/drone/imu", 10, &INPUT_HANDLER::imuDataCallback, this);

        // Best guess of initial states
        Eigen::VectorXd x0(6); // X = [vx, vy, vz, rx, ry, rz]^T
        double t = ros::Time::now().toSec();
        x0 << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
        kf.init(t, x0);
    }

private:
    void lsdPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {

        tf2::Quaternion qua_t_;
        // qua_t_.setRPY(0, 3.14159, 3.14159);
        qua_t_.setRPY(-1.5708, 0.0, 0.0);

        // Now, convert this quaternion into an Eigen rotation matrix
        Eigen::Quaterniond eigen_quat(qua_t_.w(), qua_t_.x(), qua_t_.y(), qua_t_.z());
        Eigen::Matrix3d R_world_to_openCV = eigen_quat.toRotationMatrix();

        // std::cout << "Rotation Matrix = \n"
        //           << R_world_to_openCV << std::endl;

        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();                  // Creating a 4x4 Identity matrix
        transformation_matrix.block<3, 3>(0, 0) = R_world_to_openCV;                          // Filling the rotation part of the matrix
        Eigen::Isometry3d transformation_isometry = Eigen::Isometry3d(transformation_matrix); // Converting it to Isomet

        // Convert the PoseStamped message to an Eigen pose
        Eigen::Isometry3d current_pose_eigen_lsd;
        tf2::fromMsg(msg->pose, current_pose_eigen_lsd);

        // Transform the pose to world frame
        Eigen::Isometry3d current_pose_eigen = current_pose_eigen_lsd;
        current_pose_eigen = transformation_isometry * current_pose_eigen_lsd;
        // Applying transformation

        ros::Time current_time = msg->header.stamp;

        // Compute change in pose and time
        Eigen::Isometry3d delta_pose = prev_pose_eigen.inverse() * current_pose_eigen;
        double delta_time = (current_time - prev_pose_update).toSec();

        // Compute velocity
        Eigen::Vector3d linear_velocity = delta_pose.translation() / delta_time;
        Eigen::Vector3d angular_velocity = Eigen::AngleAxisd(delta_pose.rotation()).axis() / delta_time;

        Eigen::VectorXd lsd_vel_v(3);
        lsd_vel_v << linear_velocity;

        ros::Time time = ros::Time::now();

        data_associator.addLsdData(lsd_vel_v, time);

        // Convert position to Eigen::Vector3d
        Eigen::Vector3d position = current_pose_eigen.translation() * scale_estimate;

        // Convert rotation to Eigen angles
        Eigen::Quaterniond quat = (Eigen::Quaterniond)current_pose_eigen.rotation();
        Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(2, 1, 0); // get yaw,pitch,roll (Z,Y,X)

        State vio_pose;

        vio_pose << current_pose_eigen.translation().x(),
            current_pose_eigen.translation().y(),
            current_pose_eigen.translation().z(),
            euler(2), // roll
            euler(1), // pitch
            euler(0), // yaw
            0.0f, 0.0f, 0.0f;

        vio_pose_pub.publishPose(current_pose_eigen);

        vio_euler = euler.cast<float>();

        // Concatenate position and euler angles to form 6D vector
        Eigen::VectorXd pose_eigen(6);
        pose_eigen << position, euler;

        vio_buffer_world.addPair(pose_eigen, time);

        // Store current pose and time for next velocity computation
        prev_pose_eigen = current_pose_eigen;
        prev_pose_update = current_time;
    }

    void imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {

        // Convert acceleration to Eigen vector
        Eigen::VectorXd acceleration(6);

        if (imu_to_world_trans(*msg, acceleration))
        {

            AccelerometerType acc;

            acc[0] = acceleration[0];
            acc[1] = acceleration[1];
            acc[2] = acceleration[2];
            // accelerometer_publisher.publishAccelerometer(acc);

            // Update Kalman filter with acceleration
            kf.update(acceleration);

            // Get estimated state (which includes velocity)
            Eigen::VectorXd estimated_state = kf.state();

            // Extract velocity from estimated state
            Eigen::VectorXd estimated_velocity = estimated_state.head(3); // first 3 elements are linear velocity

            // ROS_INFO_STREAM("T State: \n"
            //                 << estimated_velocity.format(Eigen::IOFormat(Eigen::FullPrecision)));

            ros::Time time = ros::Time::now();

            // DEBUG

            data_associator.addImuData(gazebo_listener.getLinearVel(), time);

            imu_buffer_world.addPair(acceleration, time);
        }
    }

public:
    bool imu_to_world_trans(sensor_msgs::Imu imu_msg, Eigen::VectorXd &tranformed)
    {
        // Define world gravity
        Eigen::Vector3f gravity_world_frame(0.0, 0.0, 9.81);

        // R for imu_to_gazebo_imu ( Ry_90 then Rx_90 -> Rx*Ry)
        Eigen::Matrix3f rawtoGazebo;
        rawtoGazebo << 0.0f, 0.0f, 1.0f,
            1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f;

        if (efk_state.size() < 6)
        {
            std::cerr << "Error: efk_state has less than 6 elements!\n";
            return false;
        }

        // Retrieve state
        Eigen::Vector3f position_world = efk_state.head<3>(); // x, y, z

        Eigen::Vector3f eulerAngles = efk_state.segment<3>(3); // roll, pitch, yaw
        Eigen::Matrix3f R_world_to_body = eulerToRotationMatrix(vio_euler);

        // std::cout << "ROll-VIO " << radToDeg(vio_euler[0]) << " PITCH-VIO " << radToDeg(vio_euler[1]) << std::endl;

        // std::cout << " " << std::endl;

        // Get the IMU data in the form of Eigen::Vector3f
        Eigen::Vector3f imu_linear_acceleration(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);

        // Compute the combined rotation from world to IMU (Note we fist re-allign the IMU vector with the gazebo vector with rawtoGazebo)
        Eigen::Matrix3f R_world_to_imu = rawtoGazebo * R_world_to_body; // Note the order of multiplication

        // Rotate gravity to the IMU frame
        Eigen::Vector3f gravity_imu_frame = R_world_to_imu * gravity_world_frame;

        // Subtract gravity from acceleration
        imu_linear_acceleration -= gravity_imu_frame;

        /** NOW WE HAVE REMOVED' THE GRAVITY VECTOR FROM THE IMU READINGS, WE CAN TRANFORM THE IMU READINGS FROM THE BODY FRAME TO THE WORLD FRAME */

        // Rotate linear acceleration from IMU frame to world frame
        Eigen::Vector3f imu_linear_acceleration_transformed = R_world_to_imu.transpose() * imu_linear_acceleration;

        // Rotate angular velocity from IMU frame to world frame
        Eigen::Vector3f imu_angular_velocity_transformed = R_world_to_imu.transpose() * Eigen::Vector3f(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);

        // Prepare result
        tranformed << imu_linear_acceleration_transformed.cast<double>(), imu_angular_velocity_transformed.cast<double>();

        return true;
    }

    Eigen::Matrix3f eulerToRotationMatrix(Eigen::Vector3f euler)
    {
        Eigen::Matrix3f R;
        // R = Eigen::AngleAxisf(euler(2), Eigen::Vector3f::UnitZ()) * // yaw
        //     Eigen::AngleAxisf(euler(1), Eigen::Vector3f::UnitY()) * // pitch
        //     Eigen::AngleAxisf(euler(0), Eigen::Vector3f::UnitX());  // roll

        R = Eigen::AngleAxisf(euler(0), Eigen::Vector3f::UnitZ()) * // yaw
            Eigen::AngleAxisf(euler(1), Eigen::Vector3f::UnitY()) * // pitch
            Eigen::AngleAxisf(euler(2), Eigen::Vector3f::UnitX());  // roll

        // R << 1.0f, 0.0f, 0.0f,
        //     0.0f, 1.0f, 0.0f,
        //     0.0f, 0.0f, 1.0f;

        return R;
    }

    double radToDeg(double radians)
    {
        if (radians > M_PI)
        {
            radians -= (2 * M_PI);
        }
        else if (radians < -M_PI)
        {
            radians += (2 * M_PI);
        }
        return radians * (180.0 / M_PI);
    }

public:
    std::deque<std::pair<Eigen::VectorXd, ros::Time>> imu_world_raw_buffer; // Stores the acc and gyro vels to be used by the efk

    DataAssociator data_associator;

    DataBuffer imu_buffer_world;

    DataBuffer vio_buffer_world;

    double scale_estimate;

    State &efk_state; // Reference to State object

private:
    ros::Subscriber lsd_pose_sub;
    ros::Subscriber imu_data_sub;

    const size_t buffer_size = 100;

    ros::Time prev_pose_update;
    Eigen::Isometry3d prev_pose_eigen;

    double dt;
    Eigen::MatrixXd A; // System dynamics matrix
    Eigen::MatrixXd C; // Output matrix
    Eigen::MatrixXd Q; // Process noise covariance
    Eigen::MatrixXd R; // Measurement noise covariance
    Eigen::MatrixXd P; // Estimate error covariance

    // Declare kf after the members it depends on
    KalmanFilter kf;

    tf2_ros::Buffer tfBuffer;

    // DEBUG

    PosePublisher vio_pose_pub;
    AccelerometerPublisher accelerometer_publisher;

    Eigen::Vector3f vio_euler;

public:
    GazeboModelListener gazebo_listener;
};
