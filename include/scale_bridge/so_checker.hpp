#pragma once
#include <deque>
#include <numeric>
#include <cmath>

/**
 * @file so_checker.hpp
 * We Analyze the convergence of the scale factor of the visual inertial system by monitoring the standard deviation of the estimated scale factor over time.
 * If the values are fluctuating but staying within a narrow range, the standard deviation will be small. 
 * Indicating the Scale factor has converged and is healthy to use
 
*/

class ScaleConvergenceChecker
{
public:
    ScaleConvergenceChecker(int buffer_size, double convergence_threshold)
        : buffer_size_(buffer_size), convergence_threshold_(convergence_threshold) {}

    void AddScaleEstimate(double estimate)
    {
        if (buffer_.size() >= buffer_size_)
        {
            buffer_.pop_front();
        }
        buffer_.push_back(estimate);
    }

    bool HasConverged() 
    {
        if (buffer_.size() < buffer_size_) return false;

        mean_ = std::accumulate(buffer_.begin(), buffer_.end(), 0.0) / buffer_.size();
        double variance = 0.0;
        for (double value : buffer_)
        {
            double diff = value - mean_;
            variance += diff * diff;
        }
        variance /= buffer_.size();
        std_dev_ = std::sqrt(variance);

        return std_dev_ < convergence_threshold_ && mean_ > 0.01;
    }

    double get_mean() 
    {
        return mean_;
    }

    double get_std_dev(){
        return std_dev_;
    }


private:
    double mean_;
    double std_dev_;
    int buffer_size_;
    double convergence_threshold_;
    std::deque<double> buffer_;
};
