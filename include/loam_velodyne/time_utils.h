#pragma once

#include <chrono>

namespace loam
{ 
  /** \brief A standard non-ROS alternative to ros::Time.*/
  using Time =
      std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;

  // helper function
  inline double toSec(Time::duration duration)
  {
    return std::chrono::duration<double>(duration).count();
  };
}
