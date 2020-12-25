#include <ros/console.h>
#include <ros/ros.h>

#include <vector>

// namespace perception_pipeline{

// }

class Statistics
{
public:
  void startCycle()
  {
    lastStartTime = ros::Time::now();
  }

  void finishCycle()
  {
    ros::Time finishTime = ros::Time::now();
    ros::Duration duration = finishTime - lastStartTime;
    cycleTimes.push_back(duration.toSec());
    cycles++;

    // ROS_INFO("Cycle took %lf secs", duration.toSec());

    if (cycles == 10)
    {
      double total;
      for (auto &n : cycleTimes)
        total += n;

      cycleTimes = {};
      cycles = 0;

      ROS_INFO("Cycle took averagely (n=10) %lf secs", total / 10);
    }
  }

private:
  std::vector<double> cycleTimes;
  int cycles = 0;
  ros::Time lastStartTime;
};
