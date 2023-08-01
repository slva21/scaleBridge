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
#include "visual_inertial/imu_fusion.hpp"
#include "data_buffer.hpp"

#include "./DataAssociator.hpp"
#include "./pose_pub.hpp"
#include "./DEBUG/acc_pub.hpp"

#include "./DEBUG/gazebo_state_sub.hpp"

#define DEG_TO_RAD 0.01745329
#define RAD_TO_DEG 57.2957786

// rosrun lsd_slam_core live_slam image:=/camera/image_raw camera_info:=/kinect/depth/camera_info

typedef Eigen::Matrix<float, 9, 1> State;

class INPUT_HANDLER
{
public:
    INPUT_HANDLER(State &state, bool _isGazebo) : dt(0.005), isGazebo(_isGazebo),
                                                  A(Eigen::MatrixXd::Zero(3, 3)),
                                                  A_gyro(Eigen::MatrixXd::Identity(3, 3)),
                                                  C(Eigen::MatrixXd::Identity(3, 3)),
                                                  Q(Eigen::MatrixXd::Identity(3, 3)),
                                                  R(Eigen::MatrixXd::Identity(3, 3)),
                                                  P(Eigen::MatrixXd::Identity(3, 3)),
                                                  efk_state(state), vio_pose_pub("visual_intertial/vio")
    {

        // Define the state transition matrix A
        // dt 0 0
        // 0 dt 0
        // 0 0 dt
        A << dt, 0.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0, dt;

        // Define the Kalman filter
        kf_acc = KalmanFilter(dt, A, C, Q, R, P);

        kf_gyro = KalmanFilter(dt, A_gyro, C, Q, R, P);

        // Initialize NodeHandle
        ros::NodeHandle nh;

        // Initialize subscribers and publishers
        // lsd_pose_sub = nh.subscribe("/lsd_slam/pose", 10, &INPUT_HANDLER::lsdPoseCallback, this);
        lsd_pose_sub = nh.subscribe("/orb_slam3/camera_pose", 10, &INPUT_HANDLER::lsdPoseCallback, this);
        imu_data_sub = nh.subscribe("/drone/imu", 10, &INPUT_HANDLER::imuDataCallback, this);

        // Best guess of initial states
        Eigen::VectorXd x0_acc(3); // X = [vx, vy, vz]^T
        double t = ros::Time::now().toSec();
        x0_acc << 0.0f, 0.0f, 0.0f;
        kf_acc.init(t, x0_acc);
        kf_gyro.init(t, x0_acc);

        kalmanZ.setRmeasure(0.03);

        // Values from IMU calibration
        double noiseGyro = 1.7e-04;   // rad/sec/sqrt(Hz)
        double noiseAcc = 2.0e-03;    // m/sec^2/sqrt(Hz)
        double gyroWalk = 1.9393e-05; // rad/sec^2/sqrt(Hz)
        double accWalk = 3.e-03;      // m/sec^3/sqrt(Hz)

        // Convert to variance values
        double Q_angle = noiseAcc * noiseAcc;     // Variance for the accelerometer
        double Q_bias = gyroWalk * gyroWalk;      // Variance for the gyro bias
        double R_measure = noiseGyro * noiseGyro; // Measurement noise variance

        kalmanX.setQangle(Q_angle);
        kalmanX.setQbias(Q_bias);
        kalmanX.setRmeasure(R_measure);

        kalmanY.setQangle(Q_angle);
        kalmanY.setQbias(Q_bias);
        kalmanY.setRmeasure(R_measure);
    }

private:
    void lsdPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {

        // Convert the PoseStamped message to an Eigen pose
        Eigen::Isometry3d current_pose_eigen_cv;
        tf2::fromMsg(msg->pose, current_pose_eigen_cv);

        // The rotation matrix and translation vector obtained from evo_ape
        Eigen::Matrix3d R_c1Tb;

        // T_C1^B = R_z(-90) * R_x(-90)
        R_c1Tb << 0.0, 0.0, 1.0,
            -1.0, 0.0, 0.0,
            0.0, -1.0, 0.0;

        // Convert the R_c1Tb and translation into a transformation matrix
        Eigen::Isometry3d T_c1Tb(Eigen::Matrix4d::Identity());
        T_c1Tb.rotate(R_c1Tb);

        // Convert the R_c1Tb and translation into a transformation matrix
        Eigen::Isometry3d T_bTw(Eigen::Matrix4d::Identity());
        T_bTw.rotate(SR_bTw);

        Eigen::Isometry3d T_cvTw = T_c1Tb.prerotate(SR_bTw); // ST_bTw * T_c1Tb;

        // Apply the T_c1Tb to the pose
        Eigen::Isometry3d current_pose_eigen = (T_cvTw * current_pose_eigen_cv);

        //Eigen::Isometry3d current_pose_eigen = current_pose_eigen_cv;

        ros::Time current_time = msg->header.stamp;

        // Compute change in pose and time
        Eigen::Isometry3d delta_pose = prev_pose_eigen * current_pose_eigen;
        double delta_time = (current_time - prev_pose_update).toSec();

        // Compute velocity
        Eigen::Vector3d linear_velocity = delta_pose.translation() / delta_time;
        Eigen::Vector3d angular_velocity = Eigen::AngleAxisd(delta_pose.rotation()).axis() / delta_time;

        Eigen::VectorXd lsd_vel_v(3);
        lsd_vel_v << linear_velocity;

        ros::Time time = ros::Time::now();

        data_associator.addLsdData(lsd_vel_v, time);

        if (scale_estimate > 0 && scale_estimate < 10)
        {
            // Convert position to Eigen::Vector3d
        }
        else
        {
            scale_estimate = 6.0;
        }

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

            // Update Kalman filter with acceleration
            kf_acc.update(acceleration.head(3)); // gets linear velocity

            // Get estimated state (which includes velocity)
            Eigen::VectorXd estimated_velocity = kf_acc.state();
            Eigen::VectorXd estimated_angular_velocity = kf_gyro.state();

            Eigen::VectorXd estimated_state(6);
            estimated_state << estimated_velocity, estimated_angular_velocity;

            // ROS_INFO_STREAM("T State: \n"
            //                 << estimated_velocity.format(Eigen::IOFormat(Eigen::FullPrecision)));

            ros::Time time = ros::Time::now();

            // RUNTIME
            data_associator.addImuData(estimated_velocity, time);
            imu_buffer_world.addPair(estimated_state, time);

            // DEBUG

            // data_associator.addImuData(gazebo_listener.getLinearVel(), time);
            // imu_buffer_world.addPair(acceleration, time);
        }
    }

public:
    bool imu_to_world_trans(sensor_msgs::Imu _imu_msg, Eigen::VectorXd &tranformed)
    {

        sensor_msgs::Imu imu_msg = tranform_EuRoC_to_ROS_frame(_imu_msg);

        // sensor_msgs::Imu imu_msg = _imu_msg;

        // Define world gravity
        Eigen::Vector3f gravity_world_frame(0.0, 0.0, -9.81);

        if (efk_state.size() < 6)
        {
            ROS_ERROR("Error: efk_state has less than 6 elements!\n");
            return false;
        }

        kf_gyro.update(Eigen::Vector3d(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z)); // angular velocity
        Eigen::VectorXd imu_angular_velocity_filtered = kf_gyro.state();

        imu_angular_velocity_filtered = Eigen::Vector3d(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);

        // Get the IMU data in the form of Eigen::Vector3 f
        Eigen::Vector3f imu_linear_acceleration(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);

        if (!SR_bTw_initialised) // We only want ot update this once at the start
        {   
            SR_bTw = get_bTw(imu_msg, imu_angular_velocity_filtered).cast<double>();
        }

        R_body_to_world = get_bTw(imu_msg, imu_angular_velocity_filtered);

        // Rotate gravity to the IMU frame
        Eigen::Vector3f gravity_imu_frame = R_body_to_world.inverse() * gravity_world_frame;

        // Subtract gravity from acceleration

        imu_linear_acceleration += gravity_imu_frame;

        // ROS_INFO_STREAM("T State: \n"
        //                 << imu_linear_acceleration.format(Eigen::IOFormat(Eigen::FullPrecision)));

        /** NOW WE HAVE REMOVED' THE GRAVITY VECTOR FROM THE IMU READINGS, WE CAN TRANFORM THE IMU READINGS FROM THE BODY FRAME TO THE WORLD FRAME */

        // Rotate linear acceleration from IMU frame to world frame
        Eigen::Vector3f imu_linear_acceleration_world = R_body_to_world.transpose() * imu_linear_acceleration;

        // Prepare result
        tranformed << imu_linear_acceleration_world.cast<double>(), imu_angular_velocity_filtered;

        return true;
    }

    Eigen::Matrix3f get_bTw(sensor_msgs::Imu imu_msg, Eigen::VectorXd imu_angular_velocity)
    {
        Eigen::Vector4d orientation_world;
        Eigen::Quaterniond quat;

        if (isGazebo)
        {
            // While we can use the Gazebo IMU for roll and pitch calculations under normal conditions, if external forces are applied to the drone via plugins, it might introduce complexities that the simulated IMU cannot handle accurately. So we just use the Gazebo pose topic instead
            orientation_world << gazebo_listener.getSafeOrientation();

            // Convert Eigen::Vector4d to Eigen::Quaterniond
            Eigen::Quaterniond quat_;
            quat = Eigen::Quaterniond(orientation_world(3), orientation_world(0), orientation_world(1), orientation_world(2)); // w, x, y, z
            quat.inverse();                                                                                                    // we wanr bTw not wTb

        }
        else
        {
            // We use the real imu data to calculate eurler orientation and then transform to quaternions
            double roll, pitch, yaw;
            yaw = 2 * M_PI;
            calc_eurler_angle(imu_msg, imu_angular_velocity, roll, pitch);

            // Create a 3D rotation matrix from roll, pitch, and yaw
            Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

            // Combine the rotations in the correct order: yaw, pitch, roll
            quat = (yawAngle * pitchAngle * rollAngle);

        }

        SR_bTw_initialised = true;

        // Convert Eigen::Quaterniond to Eigen::Matrix3d
        return quat.normalized().toRotationMatrix().cast<float>();
    }

    sensor_msgs::Imu tranform_EuRoC_to_ROS_frame(sensor_msgs::Imu imu_msg)
    {

        sensor_msgs::Imu transformed_imu;

        if (isGazebo)
        {
            // No transformation required
            transformed_imu = imu_msg;
        }
        else
        {
            // Here we manually apply (passive)rotation of the EuRoc data to match the ROS axis
            transformed_imu.linear_acceleration.x = imu_msg.linear_acceleration.z;
            transformed_imu.linear_acceleration.y = -imu_msg.linear_acceleration.y;
            transformed_imu.linear_acceleration.z = imu_msg.linear_acceleration.x;

            transformed_imu.angular_velocity.x = imu_msg.angular_velocity.z;
            transformed_imu.angular_velocity.y = -imu_msg.angular_velocity.y;
            transformed_imu.angular_velocity.z = imu_msg.angular_velocity.x;

            transformed_imu.orientation = imu_msg.orientation;
        }

        return transformed_imu;
    }

    void calc_eurler_angle(sensor_msgs::Imu imu_msg, Eigen::VectorXd &angular_vel, double &_roll, double &_pitch)
    {

        angular_vel = angular_vel;

        double pitch = atan2((-imu_msg.linear_acceleration.x), sqrt(imu_msg.linear_acceleration.y * imu_msg.linear_acceleration.y + imu_msg.linear_acceleration.z * imu_msg.linear_acceleration.z));

        double roll = atan2(imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);

        kalAngleY = kalmanY.getAngle(pitch, angular_vel[1], dt); // Calculate the angle using a Kalman filter

        kalAngleX = kalmanX.getAngle(roll, angular_vel[0], dt);

        _roll = kalAngleX;
        _pitch = kalAngleY;

        // std::cout << "Pitch:" << pitch * RAD_TO_DEG << " roll: " << roll * RAD_TO_DEG << std::endl;
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

    Eigen::Matrix3f R_body_to_world;

    /**

    @brief This matrix captures the initial orientation offset between the drone's body frame and the world frame.
    @details Notably, if the world frame is defined such that its initial orientation aligns with the drone's initial orientation, the yaw component of this offset will be zero (also the pitch and roll too is the drone was level).
    This orientation offset does not necessarily align with geographic North or a fixed reference point in the environment. It primarily corresponds to the drone's initial forward direction.
    However, if a magnetometer is introduced or if there's a need to define the world frame according to a different orientation, this offset will accommodate these changes.
    Note that this is a static rotation matrix; it captures the initial offset and does not change over time.
    */
    Eigen::Matrix3d SR_bTw;
    bool SR_bTw_initialised;

    bool isGazebo;

    ros::Time prev_pose_update;
    Eigen::Isometry3d prev_pose_eigen;

    double dt;
    Eigen::MatrixXd A;      // System dynamics matrix
    Eigen::MatrixXd A_gyro; // System dynamics matrix
    Eigen::MatrixXd C;      // Output matrix
    Eigen::MatrixXd Q;      // Process noise covariance
    Eigen::MatrixXd R;      // Measurement noise covariance
    Eigen::MatrixXd P;      // Estimate error covariance

    // Declare kf after the members it depends on
    KalmanFilter kf_acc;

    KalmanFilter kf_gyro;

    tf2_ros::Buffer tfBuffer;

    IMU_FUSION kalmanX; // Kalman filter instance for X axis
    IMU_FUSION kalmanY; // Kalman filter instance for Y axis
    IMU_FUSION kalmanZ; // Kalman filter instance for Z asis

    double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter

    std::mutex R_body_to_world_mtx; // Declare a mutex

    // DEBUG

    PosePublisher vio_pose_pub;
    AccelerometerPublisher accelerometer_publisher;

    Eigen::Vector3f vio_euler;

public:
    GazeboModelListener gazebo_listener;
};

// linear_acceleration:
//   x: -3.637751047318559
//   y: -4.81203488009604e-10
//   z: 9.099822378371442
// linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]