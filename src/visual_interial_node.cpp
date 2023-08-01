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
#include "visual_inertial/scale_optimiserv2.hpp"

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

    ros::NodeHandle nh;

    bool isGazebo = true;
    //nh.param("isGazebo", false);

    ROS_INFO("INERTIAL PLUGIN FOR ROS LSD SLAM: BY ADEBAYO, 2023, KINGS COLLEGE LONDON ");

    x0 << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;

    INPUT_HANDLER ros_lsd_imu_input(x0, isGazebo);

    ScaleEstimator scale_estimator(2000); // window size of 2000

    ScaleEstimatorV2 scale_estimatorv2(2000); // window size of 2000

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

            // scale_estimatorv2.AddDataPair(matching_data.first, matching_data.second);

            // Update the scale estimate
            scale_estimator.EstimateScale(ros_lsd_imu_input.scale_estimate);

            // double scale_v2;
            // scale_estimatorv2.EstimateScale();
            // scale_v2  = scale_estimatorv2.GetScale();

           // std::cout << "ESTIMATED SCALE: " << scale_v2 << std::endl;

            //   ROS_INFO_STREAM("mathcing data found");
        }
        ros::spinOnce();
    }

    t_ekf.join();
    return 0;
}

void RUN_EKF(INPUT_HANDLER *ros_lsd_imu_input)
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
            systemModel.dt_ = 0.001;

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
            // z[0] = z[0]*6.0;
            // z[1] = z[1]*6.0;
            // z[2] = z[2]*6.0;

            //

            Eigen::Vector3d gazebo_pos = ros_lsd_imu_input->gazebo_listener.getSafePosition();

            Eigen::Vector4d gazebo_orien = ros_lsd_imu_input->gazebo_listener.getSafeOrientation();

            Eigen::Quaterniond q(
                gazebo_orien[3],
                gazebo_orien[0],
                gazebo_orien[1],
                gazebo_orien[2]);

            Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);

            double roll = euler[2];
            double pitch = euler[1];
            double yaw = euler[0];

            // DEBUG

            MeasurementType _z;

            _z << gazebo_pos.cast<float>(), (float)roll, (float)pitch, (float)yaw;

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

// rosrun lsd_slam_core live_slam image:=/camera/image_raw camera_info:=/kinect/depth/camera_info

// rosrun lsd_slam_core live_slam image:=/cam0/image_raw _calib:=/home/sorair/catkin_ws/src/visual_intertial/lsd_slam_cfg/EuRoC.cfg

// rosrun lsd_slam_core live_slam image:=/cam0/image_raw camera_info:=/camera_info

//rosrun lsd_slam_core live_slam image:=/tello/camera _hz:=20 _calib:=/home/sorair/catkin_ws/src/visual_intertial/lsd_slam_cfg/tello.cfg

// evo_ape kitti ./gazebo_poses3.txt orb_rgbd.txt -va --correct_scale --plot


 // R_c1Tb << 0.19121822, 0.1643474, -0.96769082,
        //     0.98039522, -0.07973737, 0.18018647,
        //     -0.04754794, -0.98317439, -0.17637265;