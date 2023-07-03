#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <Eigen/Dense>

#include "./ROS/input_handler.hpp"
#include "visual_inertial/scale_optimser.hpp"
#include <thread>

#include "visual_inertial/extended_kalman_conf.hpp"
#include "ROS/pose_pub.hpp"

geometry_msgs::TransformStamped get_cam_to_body();

void test_imu_to_world_trans(INPUT_HANDLER *);

void RUN_EKF(INPUT_HANDLER *);

// Initialize the filter
Kalman::ExtendedKalmanFilter<StateType> ekf;

// Initialize the state
StateType x0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "interial_pluggin");

    ROS_INFO("INERTIAL PLUGIN FOR ROS LSD SLAM: BY ADEBAYO, 2023, KINGS COLLEGE LONDON ");

    x0 << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;

    INPUT_HANDLER ros_lsd_imu_input(x0);

    ScaleEstimator scale_estimator(20); // window size of 20

    std::thread t_ekf(RUN_EKF, &ros_lsd_imu_input); // spawn new thread that calls bar(0)

    // test_imu_to_world_trans(&ros_lsd_imu_input);

    // Loop 1: Scale Estimation Loop

    while (ros::ok())
    {
        std::pair<Eigen::VectorXd, Eigen::VectorXd> matching_data;

        if (ros_lsd_imu_input.data_associator.getMatchingData(matching_data))
        {

            // ROS_INFO_STREAM("IMU: \n"
            //                 << matching_data.second.format(Eigen::IOFormat(Eigen::FullPrecision)));

            // Pass to scale optimiser
            scale_estimator.AddDataPair(matching_data.first, matching_data.second);

            // Update the scale estimate
            ros_lsd_imu_input.scale_estimate = scale_estimator.EstimateScale();

            // std::cout << "ESTIMATED SCALE: " << ros_lsd_imu_input.scale_estimate << std::endl;

            //   ROS_INFO_STREAM("mathcing data found");
        }
        ros::spinOnce();
    }

    t_ekf.join();
    return 0;
}

void RUN_EKF(INPUT_HANDLER *ros_lsd_imu_input)
{
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
            systemModel.dt_ = 0.001;

            // Perform prediction step with the system model and control input

            // DEBUG
            Eigen::Vector3d linear_vel = ros_lsd_imu_input->gazebo_listener.getLinearVel();
            Eigen::Vector3d rot_vel = ros_lsd_imu_input->gazebo_listener.getAngularVel();

            ControlType control;
            control << linear_vel(0), linear_vel(1), linear_vel(2), rot_vel(0), rot_vel(1), rot_vel(2);
            x0 = ekf.predict(systemModel, control);
        }

        Eigen::VectorXd lsd_pos_world(6);
        double _dt;

        if (ros_lsd_imu_input->vio_buffer_world.fetchData(lsd_pos_world, _dt))
        {
            // Define measurement (from VIO)
            MeasurementType z;

            z = lsd_pos_world.cast<float>();
            // Perform update step with the measurement model and measurement
            x0 = ekf.update(measurementModel, z);
        }

        ekf_pose_pub.publishPose(x0);

        // ROS_INFO_STREAM("EKF: \n"
        //                 << x0.format(Eigen::IOFormat(Eigen::FullPrecision)));

        // DEBUG

        Eigen::Vector3d position = ros_lsd_imu_input->gazebo_listener.getSafePosition();

        Eigen::Vector4d quaternion = ros_lsd_imu_input->gazebo_listener.getSafeOrientation();

        // Convert Quaternion to Euler angles
        Eigen::Quaterniond q(quaternion(3), quaternion(0), quaternion(1), quaternion(2));
        Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX

        StateType state;
        state << position(0), position(1), position(2), euler(2), euler(1), euler(0), 0.0, 0.0, 0.0;

        gazebo_pose_pub.publishPose(state);
    }
}

// UNIT FUNCTION TESTING

void test_imu_to_world_trans(INPUT_HANDLER *ros_lsd_imu_input)
{
    // Step 1: Define Known Inputs and Expected Outputs

    // For simplicity, let's use a single test case. You should add more diverse test cases.
    sensor_msgs::Imu test_input;
    test_input.linear_acceleration.x = -9.81f; // IMU resting on a flat surface
    test_input.linear_acceleration.y = 1.0f;
    test_input.linear_acceleration.z = 2.0f;

    test_input.angular_velocity.x = 0.1f;
    test_input.angular_velocity.y = 0.2f;
    test_input.angular_velocity.z = 0.3f; // No rotation

    // Expected output when the camera to IMU transform is identity and EKF state indicates no translation or rotation.
    Eigen::VectorXd expected_output(6);
    expected_output << 0.0, 1.0, 2.0, 0.1, 0.2, 0.3; // In world frame

    // Step 2: Implement the Test Function

    Eigen::VectorXd actual_output(6);
    bool success = ros_lsd_imu_input->imu_to_world_trans(test_input, actual_output);

    // Step 3: Verify the Results

    if (!success)
    {
        std::cerr << "imu_to_world_trans failed to compute transformed IMU data.\n";
        return;
    }

    if (!actual_output.isApprox(expected_output, 1e-6))
    { // Allow for minor numerical differences
        std::cerr << "Test failed. Expected: " << expected_output.transpose() << ", but got: " << actual_output.transpose() << "\n";
        return;
    }

    std::cout << "Test passed.\n";
}
