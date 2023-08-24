#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <Eigen/Dense>

#include "./ROS/input_handler.hpp"
#include "./ROS/scale_pub.hpp"
#include "scale_bridge/scale_optimser.hpp"
#include <thread>

#include "scale_bridge/extended_kalman_conf.hpp"
#include "ROS/pose_pub.hpp"
#include "scale_bridge/so_checker.hpp"

geometry_msgs::TransformStamped get_cam_to_body();

void test_imu_to_world_trans(INPUT_HANDLER *);

void RUN_EKF(INPUT_HANDLER *, bool);

// Initialize the filter
Kalman::ExtendedKalmanFilter<StateType> ekf;

// Initialize the state
StateType x0;

int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "scale_bridge");
    ros::NodeHandle nh;

    ROS_INFO("SCALE_BRIDGE: INERTIAL PLUGIN FOR ROS MONOSLAM: BY ADEBAYO, 2023, KINGS COLLEGE LONDON ");

    int cc_bufferSize, se_bufferSize;
    double cc_thresh;
    bool useGazebo;

    nh.getParam("SE_BufferSize", se_bufferSize);
    nh.getParam("CC_BufferSize", cc_bufferSize);
    nh.getParam("CC_StdThreshhold", cc_thresh);
    nh.getParam("DEBUG_UseGazebo", useGazebo);


    x0 << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;

    INPUT_HANDLER ros_lsd_imu_input(x0, nh);

    ScaleEstimator scale_estimator(se_bufferSize);

    ScaleConvergenceChecker convergence_checker(cc_bufferSize, cc_thresh);

    ScalePublisher scale_pub;

    std::thread t_ekf(RUN_EKF, &ros_lsd_imu_input, useGazebo);

    // Loop 1: Scale Estimation Loop

    while (ros::ok())
    {
        std::pair<Eigen::VectorXd, Eigen::VectorXd> matching_data;

        if (ros_lsd_imu_input.data_associator.getMatchingData(matching_data))
        {

            // Pass to scale optimiser
            scale_estimator.AddDataPair(matching_data.first, matching_data.second);

            // Update the scale estimate
            double estimated_scale;

            scale_estimator.EstimateScale(estimated_scale);
            convergence_checker.AddScaleEstimate(estimated_scale);

            bool systemConverged = convergence_checker.HasConverged();

            // Only update the scale during convertion
            if (systemConverged)
            {
                ros_lsd_imu_input.scale_estimate = estimated_scale;
            }
            scale_pub.publishMessage(estimated_scale, systemConverged, convergence_checker.get_std_dev(), convergence_checker.get_mean());
        }
        ros::spinOnce();
    }

    t_ekf.join();
    return 0;
}

void RUN_EKF(INPUT_HANDLER *ros_lsd_imu_input, bool useGazebo)
{

    // Use gazebo position values here to test if the EKF is working as expected
    x0.setZero();

    PosePublisher ekf_pose_pub("visual_intertial/ekf");

    PosePublisher gazebo_pose_pub("visual_intertial/gazebo");

    // Define system model
    double dt = 0.01; // Time step
    IMUSystemModel systemModel(dt);

    // Define measurement model
    VIOPoseMeasurementModel measurementModel(ros_lsd_imu_input->scale_estimate);

    while (ros::ok())
    {
        Eigen::VectorXd imu_data_world(6);
        double dt;

        if (ros_lsd_imu_input->imu_buffer_world.fetchData(imu_data_world, dt))
        {
            // Define control input (from IMU readings)
            ControlType u;
            u = imu_data_world.cast<float>();
            systemModel.dt_ = ros_lsd_imu_input->dt;

            // Perform prediction step with the system model and control input

            // DEBUG
            Eigen::Vector3d linear_vel = ros_lsd_imu_input->gazebo_listener.getLinearVel();

            Eigen::Vector3d rot_vel = ros_lsd_imu_input->gazebo_listener.getAngularVel();

            ControlType control;
            control << linear_vel(0), linear_vel(1), linear_vel(2), rot_vel(0), rot_vel(1), rot_vel(2);
            x0 = ekf.predict(systemModel, u);
        }

        Eigen::VectorXd lsd_pos_world(6);
        double _dt;

        if (ros_lsd_imu_input->vio_buffer_world.fetchData(lsd_pos_world, _dt))
        {

            // Define measurement (from VIO)
            MeasurementType z;

            z = lsd_pos_world.cast<float>();

            // DEBUG

            // Eigen::Vector3d gazebo_pos = ros_lsd_imu_input->gazebo_listener.getSafePosition();

            // Eigen::Vector4d gazebo_orien = ros_lsd_imu_input->gazebo_listener.getSafeOrientation();

            // Eigen::Quaterniond q(
            //     gazebo_orien[3],
            //     gazebo_orien[0],
            //     gazebo_orien[1],
            //     gazebo_orien[2]);

            // Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);

            // double roll = euler[2];
            // double pitch = euler[1];
            // double yaw = euler[0];

            // MeasurementType _z;
            // _z << gazebo_pos.cast<float>(), (float)roll, (float)pitch, (float)yaw;
            // z = _z;

            // Perform update step with the measurement model and measurement
            x0 = ekf.update(measurementModel, z);
        }

        // DEBUG
        if (useGazebo)
        {
            Eigen::Vector3d position = ros_lsd_imu_input->gazebo_listener.getSafePosition();

            Eigen::Vector4d quaternion = ros_lsd_imu_input->gazebo_listener.getSafeOrientation();

            // Convert Quaternion to Euler angles
            Eigen::Quaterniond q(quaternion(3), quaternion(0), quaternion(1), quaternion(2));
            Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX

            StateType state;
            state << position(0), position(1), position(2), euler(2), euler(1), euler(0), 0.0, 0.0, 0.0;

            if (ros_lsd_imu_input->MonoSlamStarted)
            {
                gazebo_pose_pub.publishPose(state);
            }
        }

        if (ros_lsd_imu_input->MonoSlamStarted)
        {
            ekf_pose_pub.publishPose(x0);
        }
    }
}
