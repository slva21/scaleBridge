#pragma once

#include <ros/ros.h>
#include "scale_bridge/ScaleEstimate.h"

class ScalePublisher
{
public:
  ScalePublisher()
  {
    pub_ = nh_.advertise<scale_bridge::ScaleEstimate>("visual_inertial/scale", 10);
  }

  void publishMessage(double scale_estimate, bool converged, float std_dev, float mean)
  {
    scale_bridge::ScaleEstimate msg;
    msg.scale_estimate = scale_estimate;
    msg.converged = converged;
    msg.std_dev = std_dev;
    msg.mean = mean;
    pub_.publish(msg);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
};
