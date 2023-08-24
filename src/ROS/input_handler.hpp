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
#include "scale_bridge/single_kalman.hpp"
#include "scale_bridge/imu_fusion.hpp"
#include "data_buffer.hpp"

#include "./DataAssociator.hpp"
#include "./pose_pub.hpp"
#include "./DEBUG/acc_pub.hpp"

#include "./DEBUG/gazebo_state_sub.hpp"

#define DEG_TO_RAD 0.01745329
#define RAD_TO_DEG 57.2957786

typedef Eigen::Matrix<float, 9, 1> State;

class INPUT_HANDLER
{
public:
    INPUT_HANDLER(State &state, ros::NodeHandle &nh) : A(Eigen::MatrixXd::Zero(3, 3)),
                                                       A_gyro(Eigen::MatrixXd::Identity(3, 3)),
                                                       C(Eigen::MatrixXd::Identity(3, 3)),
                                                       Q(Eigen::MatrixXd::Identity(3, 3)),
                                                       R(Eigen::MatrixXd::Identity(3, 3)),
                                                       P(Eigen::MatrixXd::Identity(3, 3)),
                                                       efk_state(state), vio_pose_pub("visual_intertial/vio")
    {
        // Values from IMU calibration
        double noiseGyro;
        double noiseAcc;
        double gyroWalk;
        double accWalk;

        // Fetch R_c1Tb
        std::vector<double> R_c1Tb_values;

        nh.getParam("DEBUG_UseGazebo", useGazebo);
        nh.getParam("IMU_Frequency", dt);
        nh.getParam("IMU_NoiseAcc", noiseAcc);
        nh.getParam("IMU_GyroWalk", gyroWalk);
        nh.getParam("IMU_AccWalk", accWalk);
        nh.getParam("IMU_Frequency", dt);
        dt = 1 / dt;

        if (nh.getParam("R_c1Tb", R_c1Tb_values) && R_c1Tb_values.size() == 9)
        {

            R_c1Tb << R_c1Tb_values[0], R_c1Tb_values[1], R_c1Tb_values[2],
                R_c1Tb_values[3], R_c1Tb_values[4], R_c1Tb_values[5],
                R_c1Tb_values[6], R_c1Tb_values[7], R_c1Tb_values[8];
        }
        else
        {
            ROS_ERROR("Failed to load R_c1Tb from parameters!");
        }

        // Define the state transition matrix A
        A << dt, 0.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0, dt;

        // Define the Kalman filter
        kf_acc = KalmanFilter(dt, A, C, Q, R, P);

        kf_gyro = KalmanFilter(dt, A_gyro, C, Q, R, P);

        // Initialize subscribers and publishers
        lsd_pose_sub = nh.subscribe("/orb_slam2_mono/pose", 10, &INPUT_HANDLER::MonoSLAMPoseCallback, this);
        imu_data_sub = nh.subscribe("/tello/imu", 10, &INPUT_HANDLER::imuDataCallback, this);

        // Best guess of initial states
        Eigen::VectorXd x0_acc(3); // X = [vx, vy, vz]^T
        double t = ros::Time::now().toSec();
        x0_acc << 0.0f, 0.0f, 0.0f;
        kf_acc.init(t, x0_acc);
        kf_gyro.init(t, x0_acc);

        kalmanZ.setRmeasure(0.03);

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

        scale_estimate = 1;
        MonoSlamStarted = false;
    }

private:
    void MonoSLAMPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {

        MonoSlamStarted = true;

        // Convert the PoseStamped message to an Eigen pose
        Eigen::Isometry3d current_pose_eigen_cv;
        tf2::fromMsg(msg->pose, current_pose_eigen_cv);

        // Convert the R_c1Tb and translation into a transformation matrix
        Eigen::Isometry3d T_c1Tb(Eigen::Matrix4d::Identity());
        T_c1Tb.rotate(R_c1Tb);

        Eigen::Isometry3d T_cvTw = T_c1Tb.prerotate(SR_bTw.Identity()); // ST_bTw * T_c1Tb;

        // Apply the T_c1Tb to the pose
        Eigen::Isometry3d current_pose_eigen = (T_cvTw * current_pose_eigen_cv);

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
            kf_acc.update(acceleration.head(3)); // gets the new linear imu vel from the kf

            // Get estimated state (which includes velocity)
            Eigen::VectorXd estimated_velocity = kf_acc.state();
            Eigen::VectorXd estimated_angular_velocity = kf_gyro.state();

            Eigen::VectorXd estimated_state(6);
            estimated_state << estimated_velocity, estimated_angular_velocity;

            ros::Time time = ros::Time::now();

            if (useGazebo)
            {
                // DEBUG
                data_associator.addImuData(gazebo_listener.getLinearVel(), time);
                imu_buffer_world.addPair(acceleration, time);
            }
            else
            {
                // RUNTIME
                data_associator.addImuData(estimated_velocity, time);
                imu_buffer_world.addPair(estimated_state, time);
            }
        }
    }

public:
    bool imu_to_world_trans(sensor_msgs::Imu _imu_msg, Eigen::VectorXd &tranformed)
    {

        sensor_msgs::Imu imu_msg = _imu_msg;

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

        // Rotate linear acceleration from IMU frame to world frame
        Eigen::Vector3f imu_linear_acceleration_world = R_body_to_world * imu_linear_acceleration;

        // Prepare result
        tranformed << imu_linear_acceleration_world.cast<double>(), imu_angular_velocity_filtered;

        return true;
    }

    Eigen::Matrix3f get_bTw(sensor_msgs::Imu imu_msg, Eigen::VectorXd imu_angular_velocity)
    {
        Eigen::Vector4d orientation_world;
        Eigen::Quaterniond quat;

        if (useGazebo)
        {
            // While we can use the Gazebo IMU for roll and pitch calculations under normal conditions, if external forces are applied to the drone via plugins, it might introduce complexities that the simulated IMU cannot handle accurately. So we just use the Gazebo pose topic instead
            orientation_world << gazebo_listener.getSafeOrientation();

            // Convert Eigen::Vector4d to Eigen::Quaterniond
            Eigen::Quaterniond quat_;
            quat_ = Eigen::Quaterniond(orientation_world(3), orientation_world(0), orientation_world(1), orientation_world(2)); // w, x, y, z

            quat = quat_.inverse(); // we want bTw not wTb
        }
        else
        {
            // We use the real imu data to calculate eurler orientation and then transform to quaternions
            double roll, pitch, yaw;
            calc_eurler_angle(imu_msg, imu_angular_velocity, roll, pitch, yaw);

            // Create a 3D rotation matrix from roll, pitch, and yaw
            Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

            // Combine the rotations in the correct order: yaw, pitch, roll
            quat = (yawAngle * pitchAngle * rollAngle);

            // Use this tempotaily when using the tello
            quat = Eigen::Quaterniond(imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z);
        }

        SR_bTw_initialised = true;

        // Convert Eigen::Quaterniond to Eigen::Matrix3d
        return quat.normalized().toRotationMatrix().cast<float>();
    }

    sensor_msgs::Imu tranform_EuRoC_to_ROS_frame(sensor_msgs::Imu imu_msg)
    {

        sensor_msgs::Imu transformed_imu;

        // Here we manually apply (passive)rotation of the EuRoc data to match the ROS axis
        transformed_imu.linear_acceleration.x = imu_msg.linear_acceleration.z;
        transformed_imu.linear_acceleration.y = -imu_msg.linear_acceleration.y;
        transformed_imu.linear_acceleration.z = imu_msg.linear_acceleration.x;

        transformed_imu.angular_velocity.x = imu_msg.angular_velocity.z;
        transformed_imu.angular_velocity.y = -imu_msg.angular_velocity.y;
        transformed_imu.angular_velocity.z = imu_msg.angular_velocity.x;

        transformed_imu.orientation = imu_msg.orientation;

        return transformed_imu;
    }

    void calc_eurler_angle(sensor_msgs::Imu imu_msg, Eigen::VectorXd &angular_vel, double &_roll, double &_pitch, double &_yaw)
    {

        double pitch = atan2((-imu_msg.linear_acceleration.x), sqrt(imu_msg.linear_acceleration.y * imu_msg.linear_acceleration.y + imu_msg.linear_acceleration.z * imu_msg.linear_acceleration.z));

        double roll = atan2(imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);

        double yaw = vio_euler(0); // yaw

        kalAngleZ += angular_vel[2] * dt;

        if (kalAngleZ < 0)
        {
            kalAngleZ += (M_PI * 2);
        }
        else if (kalAngleZ > (M_PI * 2))
        {
            kalAngleZ -= (M_PI * 2);
        }

        // double yaw = 0;

        kalAngleY = kalmanY.getAngle(pitch, angular_vel[1], dt); // Calculate the angle using a Kalman filter

        kalAngleX = kalmanX.getAngle(roll, angular_vel[0], dt);

        _roll = kalAngleX;
        _pitch = kalAngleY;
        _yaw = kalAngleZ;
    }

public:
    std::deque<std::pair<Eigen::VectorXd, ros::Time>> imu_world_raw_buffer; // Stores the acc and gyro vels to be used by the efk

    DataAssociator data_associator;

    DataBuffer imu_buffer_world;

    DataBuffer vio_buffer_world;

    double scale_estimate;

    State &efk_state; // Reference to State object

    bool MonoSlamStarted;

private:
    ros::Subscriber lsd_pose_sub;
    ros::Subscriber imu_data_sub;

    const size_t buffer_size = 100;

    Eigen::Matrix3f R_body_to_world;

    Eigen::Matrix3d R_c1Tb;

    /**

    @brief This matrix captures the initial orientation offset between the drone's body frame and the world frame.
    @details Notably, if the world frame is defined such that its initial orientation aligns with the drone's initial orientation, the yaw component of this offset will be zero (also the pitch and roll too is the drone was level).
    This orientation offset does not necessarily align with geographic North or a fixed reference point in the environment. It primarily corresponds to the drone's initial forward direction.
    However, if a magnetometer is introduced or if there's a need to define the world frame according to a different orientation, this offset will accommodate these changes.
    Note that this is a static rotation matrix; it captures the initial offset and does not change over time.
    */
    Eigen::Matrix3d SR_bTw;
    bool SR_bTw_initialised;

    bool useGazebo;

    ros::Time prev_pose_update;
    Eigen::Isometry3d prev_pose_eigen;

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
    double dt; // IMU Update rate
};
