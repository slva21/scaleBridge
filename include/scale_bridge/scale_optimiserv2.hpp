#pragma once

#include <deque>
#include <ceres/ceres.h>
#include <Eigen/Core>

// Creates and solves a Ceres problem that finds the scale factor which best aligns the velocities. The scale factor is constrained to be between 0 and a maximum scale factor. 

struct CostFunctor {
    CostFunctor(double vio_velocity, double imu_velocity) 
        : vio_velocity_(vio_velocity), imu_velocity_(imu_velocity) {}
    
    template <typename T>
    bool operator()(const T* const scale, T* residual) const {
        residual[0] = scale[0]*vio_velocity_ - imu_velocity_;
        return true;
    }

private:
    const double vio_velocity_;
    const double imu_velocity_;
};

class ScaleEstimatorV2 {
public:
    ScaleEstimatorV2(int buffer_size) : buffer_size_(buffer_size), scale_(1.0) {}

    void AddDataPair(const Eigen::VectorXd &imu_velocity, const Eigen::VectorXd &vio_velocity) {
        double velocity_threshold = 0.1;

        if (imu_velocity.norm() < velocity_threshold) {
            return;
        }

        if (imu_buffer_.size() >= buffer_size_) {
            imu_buffer_.pop_front();
            vio_buffer_.pop_front();
        }

        imu_buffer_.push_back(imu_velocity);
        vio_buffer_.push_back(vio_velocity);
    }

    bool EstimateScale() {
        if (imu_buffer_.size() < buffer_size_ || vio_buffer_.size() < buffer_size_) {
            return false;
        }

        ceres::Problem problem;

        for (int i = 0; i < buffer_size_; ++i) {
            ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(
                    new CostFunctor(vio_buffer_[i].norm(), imu_buffer_[i].norm()));
            problem.AddResidualBlock(cost_function, NULL, &scale_);
        }

        problem.SetParameterLowerBound(&scale_, 0, 0.0);
        problem.SetParameterUpperBound(&scale_, 0, maximum_scale_);

        ceres::Solver::Options options;
        ceres::Solver::Summary summary;
        options.linear_solver_type = ceres::DENSE_QR;
        ceres::Solve(options, &problem, &summary);

        return summary.IsSolutionUsable();
    }

    double GetScale() const { return scale_; }

private:
    std::deque<Eigen::VectorXd> imu_buffer_;
    std::deque<Eigen::VectorXd> vio_buffer_;
    int buffer_size_;
    double scale_;
    double maximum_scale_ = 10.0;  // Set your maximum velocity
};
