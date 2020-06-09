#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>

using namespace std;

namespace urm_37_40_node {

const static float MIN_DISTANCE = 5;
const static float MAX_DISTANCE = 30;

class Sonar {
 public:
  Sonar(int t, int e) : trigger_(t), echo_(e) {
  }

  float distance(bool* error) {
	    return 0;
  }

    *error = false;
    return 5.0;
  }

private:
};

}

int main(int argc, char **argv) {

  // Start ROS node.
  ROS_INFO("Starting node");
  ros::init(argc, argv, "urm_37_40");
  ros::NodeHandle node;
  ros::Rate rate(10);  // 10 hz

  vector<urm_37_40_node::Sonar> sonars;
  sonars.push_back(urm_37_40_node::Sonar(24, 25));

  // Build a publisher for each sonar.
  vector<ros::Publisher> sonar_pubs;
  for (int i = 0; i < sonars.size(); ++i) {
    stringstream ss;
    ss << "sonar_" << i;
    sonar_pubs.push_back(node.advertise<sensor_msgs::Range>(ss.str(), 10));
  }

  // Build base range message that will be used for
  // each time a msg is published.
  sensor_msgs::Range range;
  range.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range.min_range = 0.0;
  range.max_range = hc_sr04_node::MAX_DISTANCE;

  float distance;
  bool error;
  while(ros::ok()) {
    for (int i = 0; i < sonars.size(); ++i) {
      range.header.stamp = ros::Time::now();
      range.range = sonars[i].distance(&error);
      if (error)
	    ROS_WARN("Error on sonar %d", i);
      else
	    sonar_pubs[i].publish(range);
    }
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}