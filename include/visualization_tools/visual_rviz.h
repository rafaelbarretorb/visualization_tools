// Copyright

#ifndef VISUALIZATION_TOOLS_VISUAL_RVIZ_H_  //NOLINT
#define VISUALIZATION_TOOLS_VISUAL_RVIZ_H_

#include <ros/ros.h>

// ROS message types
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>


#include <goals_sequence_path_planner/PathArray.h>

#include <vector>
#include <string>

namespace visualization_tools {
/**
 * @class VisualRviz
 * @brief Handle all Rviz Visualization
 */
class VisualRviz {
 public:
  /**
   * @brief Default constructor of the class
   */
  explicit VisualRviz(ros::NodeHandle *nodehandle);

  /**
   * @brief Visualization of the rectangular mission area boundaries
   * pD       pC
   * 
   * pA       pB
   * @param 
   */
  void globalPlannerCb(const goals_sequence_path_planner::PathArray &paths);

  void globalPlannerPaths(const goals_sequence_path_planner::PathArray &paths,
                          int marker_lifetime);

  void deleteMarkersCb(const std_msgs::Int32 &n);

 private:
  ros::NodeHandle nh_;

  ros::Publisher global_planner_marker_pub_;

  visualization_msgs::Marker global_path_marker_;

  bool global_path_marker_initialized_;

  std::string global_frame_;

  goals_sequence_path_planner::PathArray global_paths_;

  ros::Subscriber global_paths_sub_;
  ros::Subscriber del_markers_sub_;
};

};  // namespace visualization_tools

#endif  // VISUALIZATION_TOOLS_VISUAL_RVIZ_H_
