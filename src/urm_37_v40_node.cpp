#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include "urm_37_v40/serial.h"

using namespace std;

namespace urm_37_40_node {

const static float MIN_DISTANCE = 0.5;
const static float MAX_DISTANCE = 5;

class Sonar {
 public:
  Sonar(std::string serial_name) : serial_name_(serial_name) {
    serial_ = serial_new();

    if (serial_open(serial_, "/dev/ttyUSB0", 9600) < 0) {
        ROS_ERROR("serial_open(): %s\n", serial_errmsg(serial_));
        exit(1);
    }
  }

  float distance(bool* error) {
    *error = false;

    if(serial_write(serial_, range_cmd, 4)<0){
        *error = true;
        return -1;
    }

    if(serial_read(serial_, sonar_data, 4, 100)<0){
        *error = true;
        return -1;
    };

    if(sonar_data[1] == 0xFF && sonar_data[2] == 0xFF){
        *error = true;
        return -1;
    }
    int sonarValue = sonar_data[1] << 8;
    sonarValue = sonarValue + sonar_data[2];

    return sonarValue / 1000;
  }

  void close(){
    serial_close(serial_);
    serial_free(serial_);
  };

  private:
    std::string serial_name_;
    serial_t *serial_;
    uint8_t sonar_data[4];
    const uint8_t range_cmd[4]={0x22, 0x00, 0x00, 0x22};
};

} // namespace urm_37_40_node

int main(int argc, char **argv) {

  // Start ROS node.
  ROS_INFO("Starting node");
  ros::init(argc, argv, "urm_37_40");
  ros::NodeHandle node;
  ros::Rate rate(10);  // 10 hz

  vector<urm_37_40_node::Sonar> sonars;
  sonars.push_back(urm_37_40_node::Sonar("/dev/ttyUSB0"));

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
  range.min_range = urm_37_40_node::MIN_DISTANCE;
  range.max_range = urm_37_40_node::MAX_DISTANCE;

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

  for (int i = 0; i < sonars.size(); ++i) {
    sonars[i].close();
  }
  return 0;
}