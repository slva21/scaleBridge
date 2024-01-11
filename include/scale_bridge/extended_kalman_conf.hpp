#pragma once

/**
 * @file extended_kalman_conf.hpp
  IMPLEMENTATION OF THE EKF DEFINED USING THE STATE MODEL [x, y, z, roll, pitch, yaw, vx, vy, vz]^T. WE TAKE THE u VECTOR TO BE THE READINGS FROM THE ACCELEROMETER(ABIT UNCONVENTIONAL)
  BY ADEBAYO BY ADEBAYO, 2023, KINGS COLLEGE LONDON

*/

#include "kalman/ExtendedKalmanFilter.hpp"
#include "kalman/SystemModel.hpp"
#include "kalman/MeasurementModel.hpp"
#include "kalman/LinearizedSystemModel.hpp"

typedef Eigen::Matrix<float, 9, 1> StateType; // X = [x, y, z, roll, pitch, yaw, vx, vy, vz]^T

typedef Eigen::Matrix<float, 6, 1> ControlType; // u = [ax, ay, az, rx, ry, rz]^T (acc,gyro)

typedef Eigen::Matrix<float, 6, 1> MeasurementType; // we assume our VIO provides 6D pose (3D position + 3D orientation)

// 'x' = FX + Bu + w

class IMUSystemModel : public Kalman::LinearizedSystemModel<StateType, ControlType>
{
public:
    IMUSystemModel(double dt) : dt_(dt)
    {
        Q = Kalman::SquareMatrix<float, 9>::Zero(); // Set Q to be a square matrix of the same dimension as your state vector, initializing all entries to zero.
        Q.diagonal() << 1e-4 * dt, 1e-4 * dt, 1e-4 * dt, 1e-4 * dt, 1e-4 * dt, 1e-4 * dt, 1e-4 * dt, 1e-4 * dt, 1e-4 * dt;
        this->setCovariance(Q); // TODO: This may  need to be updated at each time step
    }

    // Implementation of the system model here based on IMU readings
    // This function returns the predicted state
    StateType f(const StateType &x, const ControlType &u) const override
    {
        StateType x_pred = x; // Create a copy of the current state vector

        // x.head<3>() returns the first three elements of the state vector x, which represent the current position (x, y, z)
        // x.segment<6, 3>() returns three elements of the state vector x starting from the 6th index (0-based index), which represents the current velocity (vx, vy, vz).
        x_pred.head<3>() = x.head<3>() + dt_ * x.segment(6, 3); // new position = old position + velocity*dt

        // Update the velocities based on the acceleration (the control input)
        x_pred.segment(6, 3) = x.segment(6, 3) + dt_ * u.head<3>(); // new velocity = old velocity + acceleration*dt

        // Orientation update (assuming small angles)
        x_pred.segment(3, 3) = x.segment(3, 3) + dt_ * u.tail<3>(); // new orientation = old orientation + angular_velocity*dt

        return x_pred;
    }

    void updateJacobians(const StateType &x, const ControlType &u) override
    {
        
        // Set all elements of F to zero
        this->F.setZero();

        // d(f_x)/d(x), d(f_y)/d(y), d(f_z)/d(z)
        this->F(0, 0) = 1;
        this->F(1, 1) = 1;
        this->F(2, 2) = 1;

        // d(f_roll)/d(roll), d(f_pitch)/d(pitch), d(f_yaw)/d(yaw)
        this->F(3, 3) = dt_;
        this->F(4, 4) = dt_;
        this->F(5, 5) = dt_;

        // d(f_x)/d(vx), d(f_y)/d(vy), d(f_z)/d(vz)
        this->F(0, 6) = dt_;
        this->F(1, 7) = dt_;
        this->F(2, 8) = dt_;

        // d(f_vx)/d(vx), d(f_vy)/d(vy), d(f_vz)/d(vz)
        this->F(6, 6) = 1;
        this->F(7, 7) = 1;
        this->F(8, 8) = 1;

        // process noise
        this->W.setIdentity();
    }

public:
    double dt_;
    Kalman::SquareMatrix<float, 9> Q;
};

class VIOPoseMeasurementModel : public Kalman::LinearizedMeasurementModel<StateType, MeasurementType>
{
public:
    VIOPoseMeasurementModel(double _scale_factor) : scale_factor(_scale_factor)
    {
        R = Kalman::SquareMatrix<float, 6>::Zero(); 
        R.diagonal() << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
        this->setCovariance(R);
    }
    // Implement  of the measurement model here based on VIO pose
    // This function returns the predicted measurement
    MeasurementType h(const StateType &x) const override
    {
        MeasurementType z_pred;

        // Measurement is the position and orientation
        z_pred.head<3>() = x.head<3>();     // position
        z_pred.tail<3>() = x.segment(3, 3); // orientation (in eurler form)

        return z_pred;
    }

    void updateJacobians(const StateType &x) override

    {
        // Assuming that our measurements are simply the position and orientation, the Jacobian of the
        // measurement function with respect to the state is an identity matrix for those variables and zero elsewhere.

        // Set all elements of H to zero
        this->H.setZero();

        // d(h_x)/d(x), d(h_y)/d(y), d(h_z)/d(z)
        this->H(0, 0) = 1;
        this->H(1, 1) = 1;
        this->H(2, 2) = 1;

        // d(h_roll)/d(roll), d(h_pitch)/d(pitch), d(h_yaw)/d(yaw)
        this->H(3, 3) = 1;
        this->H(4, 4) = 1;
        this->H(5, 5) = 1;
    }

public:
    double scale_factor;
    Kalman::SquareMatrix<float, 6> R;
};
