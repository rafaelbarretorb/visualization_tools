

#include <ros/ros.h>

#include <goals_sequence_path_planner/PathArray.h>
#include <std_msgs/Int32.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "visual_topics_advertiser");
  // ros::NodeHandle nh("~");
  ros::NodeHandle nh;

  ros::Publisher global_planner_pub = nh.advertise<goals_sequence_path_planner::PathArray>(
    "global_planner", 100);

  ros::Publisher delete_markers_pub = nh.advertise<std_msgs::Int32>(
    "delete_markers", 10);

  ros::spin();
}
