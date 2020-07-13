#include <visualization_tools/visual_rviz.h>


namespace visualization_tools {

  VisualRviz::VisualRviz(ros::NodeHandle* nodehandle) : nh_(*nodehandle) {
    nh_.param("move_base/global_costmap/global_frame", global_frame_, std::string("/map"));

    // Markers Publishers
    global_planner_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("global_paths", 100);
    reference_planner_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("reference_path", 100);

    // Markers bool initializers
    global_path_marker_initialized_ = false;
    reference_path_marker_initialized_ = false;

    global_paths_sub_ = nh_.subscribe("global_planner",
                                      100,
                                      &VisualRviz::globalPlannerCb,
                                      this);

    reference_path_sub_ = nh_.subscribe("reference_planner",
                                      100,
                                      &VisualRviz::referencePathCb,
                                      this);

    del_markers_sub_ = nh_.subscribe("delete_markers", 1, &VisualRviz::deleteMarkersCb, this);
  }

  void VisualRviz::globalPlannerCb(const goals_sequence_path_planner::PathArray &paths) {
    global_paths_ = paths;
    globalPlannerPaths(global_paths_, 0);  // 0 is forever
  }

  void VisualRviz::globalPlannerPaths(const goals_sequence_path_planner::PathArray &paths,
                                      int marker_lifetime) {
    std::vector<geometry_msgs::Point> points_list;

    for (int i = 0; i < paths.paths.size(); ++i) {
      for (int j = 0; j < paths.paths[i].poses.size(); ++j) {
        geometry_msgs::Point p;
        p.x = paths.paths[i].poses[j].position.x;
        p.y = paths.paths[i].poses[j].position.y;
        points_list.push_back(p);
      }
    }

    if (!global_path_marker_initialized_) {
      global_path_marker_initialized_ = true;
      global_path_marker_.ns = "visualization_tools";
      global_path_marker_.header.frame_id = "map";
      global_path_marker_.header.stamp = ros::Time();
      global_path_marker_.type = visualization_msgs::Marker::LINE_STRIP;
      global_path_marker_.action = visualization_msgs::Marker::ADD;
      global_path_marker_.scale.x = 0.05;
      global_path_marker_.scale.y = 0.05;
      global_path_marker_.scale.z = 0.05;
      global_path_marker_.color.a = 0.3;  // Don't forget to set the alpha!
      global_path_marker_.color.r = 0.0;
      global_path_marker_.color.g = 1.0;
      global_path_marker_.color.b = 0.0;
      global_path_marker_.id = 0;
    }
    global_path_marker_.lifetime = ros::Duration(marker_lifetime);
    global_path_marker_.points = points_list;
    global_planner_marker_pub_.publish(global_path_marker_);
  }

  void VisualRviz::referencePathCb(const geometry_msgs::PoseArray &path) {
    reference_path_ = path;
    referencePath(reference_path_, 0);  // 0 is forever
  }

  void VisualRviz::referencePath(const geometry_msgs::PoseArray &path,
                          int marker_lifetime) {
      std::vector<geometry_msgs::Point> points_list;

    for (int i = 0; i < path.poses.size(); ++i) {
      geometry_msgs::Point p;
      p.x = path.poses[i].position.x;
      p.y = path.poses[i].position.y;
      points_list.push_back(p);
    }

    if (!reference_path_marker_initialized_) {
      reference_path_marker_initialized_ = true;
      reference_path_marker_.ns = "visualization_tools";
      reference_path_marker_.header.frame_id = "map";
      reference_path_marker_.header.stamp = ros::Time();
      reference_path_marker_.type = visualization_msgs::Marker::LINE_STRIP;
      reference_path_marker_.action = visualization_msgs::Marker::ADD;
      reference_path_marker_.scale.x = 0.02;
      reference_path_marker_.scale.y = 0.02;
      reference_path_marker_.scale.z = 0.02;
      reference_path_marker_.color.a = 0.75;  // Don't forget to set the alpha!
      reference_path_marker_.color.r = 1.0;
      reference_path_marker_.color.g = 0.0;
      reference_path_marker_.color.b = 0.0;
      reference_path_marker_.id = 0;
    }
    reference_path_marker_.lifetime = ros::Duration(marker_lifetime);
    reference_path_marker_.points = points_list;
    reference_planner_marker_pub_.publish(reference_path_marker_);
                          }

  void VisualRviz::deleteMarkersCb(const std_msgs::Int32 &n) {
    ROS_INFO_STREAM("Delete markers!");
    globalPlannerPaths(global_paths_, 1);
  }
};  // namespace visualization_tools
