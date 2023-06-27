#pragma once

#include <Eigen/Dense>
#include <deque>

// BY ADEBAYO BY ADEBAYO, 2023, KINGS COLLEGE LONDON


class ScaleEstimator
{
public:
    ScaleEstimator(int buffer_size) : buffer_size_(buffer_size) {}

    void AddDataPair(const Eigen::VectorXd& imu_velocity, const Eigen::VectorXd& vio_velocity)
    {
        if (imu_buffer_.size() >= buffer_size_)
        {
            imu_buffer_.pop_front();
            vio_buffer_.pop_front();
        }
        
        imu_buffer_.push_back(imu_velocity);
        vio_buffer_.push_back(vio_velocity);
    }

    double EstimateScale()
    {
        int N = imu_buffer_.size();
        Eigen::MatrixXd A(N, 1);
        Eigen::VectorXd b(N);

        for (int i = 0; i < N; ++i)
        {
            A(i, 0) = imu_buffer_[i].norm(); // Use the magnitude of imu velocity
            b(i) = vio_buffer_[i].norm();   // Use the magnitude of vio velocity
        }

        Eigen::VectorXd x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b); // Solve Ax = b

        return x(0); // The first element in x should be the scale factor
    }

private:
    std::deque<Eigen::VectorXd> imu_buffer_;
    std::deque<Eigen::VectorXd> vio_buffer_;
    int buffer_size_;
};
