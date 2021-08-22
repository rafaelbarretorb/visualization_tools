#include <visualization_tools/visual_rviz.h>


namespace visualization_tools {

VisualRviz::VisualRviz(ros::NodeHandle* nodehandle) : nh_(*nodehandle) {
  nh_.param("move_base/global_costmap/global_frame", global_frame_, std::string("/map"));

  // Markers Publishers
  global_planner_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("global_paths", 100);
  reference_planner_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("reference_path", 100);
  goal_positions_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("goal_positions", 100);

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
  goalsPositions(global_paths_, 0);
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

void VisualRviz::goalsPositions(const goals_sequence_path_planner::PathArray &paths,
                                int marker_lifetime) {
  goal_positions_marker_.markers.clear();
  // marker id
  int id = 0;
  int paths_size = paths.paths.size();
  ROS_INFO("Paths size: %d", paths_size);
  if (paths.paths.size() >= 2) {
    for (int i = 0; i < paths.paths.size() - 1; ++i) {
      int last = paths.paths[i].poses.size() - 1;
      visualization_msgs::Marker goal_marker;

      // Marker Position
      goal_marker.pose.position.x = paths.paths[i].poses[last].position.x;
      goal_marker.pose.position.y = paths.paths[i].poses[last].position.y;
      goal_marker.pose.position.z = 0.05;

      goal_marker.pose.orientation.x = 0.0;
      goal_marker.pose.orientation.y = 0.0;
      goal_marker.pose.orientation.z = 0.0;
      goal_marker.pose.orientation.w = 0.0;

      ROS_INFO("Goal %d: (%f , %f)", id, goal_marker.pose.position.x, goal_marker.pose.position.y);

      goal_marker.lifetime = ros::Duration(0);
      goal_marker.id = id++;
      goal_marker.ns = "visualization_tools";
      goal_marker.header.frame_id = "map";
      goal_marker.header.stamp = ros::Time();
      goal_marker.type = visualization_msgs::Marker::CYLINDER;
      goal_marker.action = visualization_msgs::Marker::ADD;
      goal_marker.scale.x = 0.2;
      goal_marker.scale.y = 0.2;
      goal_marker.scale.z = 0.05;
      goal_marker.color.a = 0.8;  // Don't forget to set the alpha!

      // Black color
      goal_marker.color.r = 0.0;
      goal_marker.color.g = 0.0;
      goal_marker.color.b = 0.0;

      // Add to vector
      goal_positions_marker_.markers.push_back(goal_marker);
    }
    int markers_size = goal_positions_marker_.markers.size();
    ROS_INFO("Markers size: %d", markers_size);
    goal_positions_pub_.publish(goal_positions_marker_);      
  }
}
};  // namespace visualization_tools
