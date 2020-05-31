

#include <ros/ros.h>
#include <visualization_tools/visual_rviz.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "marker_publisher");
  ros::NodeHandle nh;

  visualization_tools::VisualRviz visual_rviz(&nh);

  ros::spin();
  return 0;
}
