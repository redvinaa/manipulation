#include <thread>

#include "bin_picking/find_grasp_pose.hpp"
#include "bin_picking/utils.hpp"

#include <rclcpp/executors.hpp>
#include <rclcpp/callback_group.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/common/io.h>  // for concatenatePointCloud

#include <message_filters/sync_policies/approximate_time.h>


namespace bin_picking
{

using namespace std::chrono_literals;

FindGraspPose::FindGraspPose()
{
  // Create ROS2 node
  node_ = rclcpp::Node::make_shared("find_grasp_pose");
  mgi_node_ = rclcpp::Node::make_shared("find_grasp_pose_mgi");
  vis_node_ = rclcpp::Node::make_shared("find_grasp_pose_vis");
  psm_node_ = rclcpp::Node::make_shared("find_grasp_pose_psm");
  executor_.add_node(node_);
  executor_.add_node(mgi_node_);
  executor_.add_node(vis_node_);
  executor_.add_node(psm_node_);
  spin_thread_ = std::thread([this]() { executor_.spin(); });

  // Declare parameter for pointcloud topics
  node_->declare_parameter("pointcloud_topic_left", "");
  node_->declare_parameter("pointcloud_topic_right", "");
  pointcloud_topic_left_ = node_->get_parameter("pointcloud_topic_left").as_string();
  pointcloud_topic_right_ = node_->get_parameter("pointcloud_topic_right").as_string();
  node_->declare_parameter("camera_frame_left", "");
  node_->declare_parameter("camera_frame_right", "");
  camera_frame_left_ = node_->get_parameter("camera_frame_left").as_string();
  camera_frame_right_ = node_->get_parameter("camera_frame_right").as_string();

  node_->declare_parameter<std::string>("target_frame", "world");
  target_frame_ = node_->get_parameter("target_frame").as_string();

  node_->declare_parameter<float>("voxel_size", 0.01f);
  voxel_size_ = node_->get_parameter("voxel_size").as_double();

  node_->declare_parameter<std::string>("robot_description", "");

  /* Initialize visualization tools
   * (we need this trick to make remote control work,
   * because it needs a node that's already spinning here
   * in the constructor)
   */
  visual_tools_ = std::make_shared<rviz_visual_tools::RvizVisualTools>(
    target_frame_, "/rviz_visual_tools", vis_node_);
  visual_tools_->loadRemoteControl();
  visual_tools_->deleteAllMarkers();

  // Move group
  move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    mgi_node_, "ur_arm");
  move_group_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    mgi_node_, "gripper");

  // Robot model loader
  robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
    node_, "robot_description");

  // Initialize PlanningSceneMonitor
  psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
    psm_node_, robot_model_loader_);
  psm_->startSceneMonitor("/monitored_planning_scene");

  // Subscribers (must use shared_from_this for lifecycle safety)
  sub_cloud1_.subscribe(node_, pointcloud_topic_left_);
  sub_cloud2_.subscribe(node_, pointcloud_topic_right_);

  // Define sync policy
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> MySyncPolicy;
  sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), sub_cloud1_, sub_cloud2_);
  sync_->registerCallback(std::bind(
    &FindGraspPose::pointCloudCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void FindGraspPose::run()
{
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

void FindGraspPose::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg_left,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg_right)
{
  static bool first_time = true;
  if (first_time) {
    first_time = false;
    return;  // skip first callback to allow TF buffer to fill
    // TODO better solution?
  }

  MeasureExecutionTimeScoped timer("pointCloudCallback");
  visual_tools_->deleteAllMarkers();

  RCLCPP_INFO(
    node_->get_logger(), "==== Received clouds with %u + %u points ====",
    msg_left->width * msg_left->height,
    msg_right->width * msg_right->height);

  // Process both clouds in parallel: convert to PCL and compute normals
  std::vector<std::pair<sensor_msgs::msg::PointCloud2, std::string>> msg_frame_pairs = {
    {*msg_left,  camera_frame_left_},
    {*msg_right, camera_frame_right_}
  };

  // Launch async tasks
  std::vector<std::future<pcl::PointCloud<pcl::PointNormal>::Ptr>> futures;
  futures.reserve(msg_frame_pairs.size());

  // Note: this is not the nicest code I've ever written.
  // The problem is that when we receive the pcl here, it's already transformed
  // to world frame, but we still need to know the camera frame for the viewpoint,
  // to know which direction the normals should point.
  for (const auto &[msg, frame] : msg_frame_pairs) {
    futures.push_back(std::async(std::launch::async, [this, msg, frame]() {
      // Convert ROS msg -> PCL cloud
      MeasureExecutionTime timer_conv("fromROSMsg");
      auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      pcl::fromROSMsg(msg, *cloud);
      timer_conv.stop();

      // Get camera viewpoint in target frame
      geometry_msgs::msg::Point viewpoint;
      std::string frame_or = frame.empty() ? msg.header.frame_id : frame;
      try {
        auto tf = tf_buffer_->lookupTransform(target_frame_, frame_or, rclcpp::Time(0));
        viewpoint.x = tf.transform.translation.x;
        viewpoint.y = tf.transform.translation.y;
        viewpoint.z = tf.transform.translation.z;
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(node_->get_logger(), "Could not get camera_link position: %s", ex.what());
        viewpoint.x = viewpoint.y = viewpoint.z = 0.0;
      }

      // Compute normals
      MeasureExecutionTime timer_normals("computeNormals");
      auto cloud_with_normals = computeNormals(cloud, viewpoint);
      timer_normals.stop();

      return cloud_with_normals;
    }));
  }

  // Collect results
  std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> clouds_with_normals;
  clouds_with_normals.reserve(futures.size());
  for (auto &fut : futures)
    clouds_with_normals.push_back(fut.get());

  // Merge clouds
  MeasureExecutionTime timer_merge("mergeClouds");
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_merged(new pcl::PointCloud<pcl::PointNormal>());
  for (const auto &cloud : clouds_with_normals) {
    *cloud_merged += *cloud;
  }
  timer_merge.stop();
  RCLCPP_INFO(
    node_->get_logger(), "Merged cloud contains %zu points",
    cloud_merged->size());

  // Voxelize merged cloud
  pcl::VoxelGrid<pcl::PointNormal> voxel_filter;
  voxel_filter.setInputCloud(cloud_merged);
  voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointNormal>());
  voxel_filter.filter(*cloud_voxel);

  RCLCPP_INFO(
    node_->get_logger(), "Downsampled cloud contains %zu points after voxel grid size %.3f",
    cloud_voxel->size(), voxel_size_);

  // Visualize normals
  for (size_t i = 0; i < cloud_voxel->size(); i += 1) {  // visualize every 10th normal
    const auto & pt = cloud_voxel->points[i];
    if (!std::isnan(pt.normal_x) && !std::isnan(pt.normal_y) && !std::isnan(pt.normal_z)) {
      Eigen::Vector3d start(pt.x, pt.y, pt.z);
      Eigen::Vector3d end(pt.x + 0.01 * pt.normal_x,
                          pt.y + 0.01 * pt.normal_y,
                          pt.z + 0.01 * pt.normal_z);

      // Color based on curvature
      rviz_visual_tools::Colors color;
      if (pt.curvature < 0.001)
        color = rviz_visual_tools::GREEN;
      else if (pt.curvature < 0.01)
        color = rviz_visual_tools::YELLOW;
      else
        color = rviz_visual_tools::RED;

      visual_tools_->publishArrow(
        tf2::toMsg(start), tf2::toMsg(end), color,
        rviz_visual_tools::Scales::XXXXSMALL);
    }
  }
  visual_tools_->trigger();

  // Compute principal curvatures
  MeasureExecutionTime timer_pcs("computePrincipalCurvatures");
  auto principal_curvatures = computePrincipalCurvatures(cloud_voxel);
  timer_pcs.stop();

  static const size_t n_valid_candidates = 5;
  std::unique_ptr<CandidateGrasp> best_candidate;
  size_t n_candidates = 0;
  while (rclcpp::ok()) {
    const auto candidate = sampleCandidateGrasp(cloud_voxel, principal_curvatures, true);
    if (!candidate.success) {
      continue;
    }

    if (!best_candidate || candidate.cost < best_candidate->cost) {
      best_candidate = std::make_unique<CandidateGrasp>(candidate);
    }

    if (++n_candidates >= n_valid_candidates) {
      break;
    }
  }
  if (!best_candidate) {
    RCLCPP_WARN(node_->get_logger(), "No valid grasp candidate found");
    return;
  }

  RCLCPP_INFO(
    node_->get_logger(), "Best candidate inliers: %zu, cost: %.3f",
    best_candidate->inliers, best_candidate->cost);
  visual_tools_->deleteAllMarkers();

  auto kinematic_state = getRobotStateForPose(best_candidate->pose, 0.0);
  visualizeGripper(kinematic_state, rviz_visual_tools::Colors::GREEN);

  visual_tools_->prompt("Press 'next' in the RvizVisualToolsGui to continue");

  // // Open gripper and move to pose
  // move_group_gripper_->setNamedTarget("open");
  // move_group_gripper_->move();

  // visual_tools_->prompt("Press 'next' in the RvizVisualToolsGui to move the arm to the grasp pose");

  // move_group_arm_->setPoseTarget(
  //   tf2::toMsg(best_candidate->pose),
  //   "robotiq_85_base_link");
  // moveit::planning_interface::MoveGroupInterface::Plan plan;
  // auto error_code = move_group_arm_->plan(plan);
  // if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
  //   RCLCPP_WARN(node_->get_logger(), "Failed to plan to grasp pose");
  //   return;
  // }

  // error_code = move_group_arm_->execute(plan);
  // if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
  //   RCLCPP_WARN(node_->get_logger(), "Failed to execute plan to grasp pose");

  //   // Try to move back to home
  //   move_group_arm_->setNamedTarget("home");
  //   move_group_arm_->move();

  //   return;
  // }

  // // Close gripper
  // move_group_gripper_->setJointValueTarget(
  //   "robotiq_85_left_knuckle_joint", 0.8);  // 0.8 = closed
  // move_group_gripper_->move();

  // // Try to move back to home
  // move_group_arm_->setNamedTarget("home");
  // move_group_arm_->move();

  // RCLCPP_INFO(node_->get_logger(), "Grasp pose execution complete.");
  // visual_tools_->prompt("Press 'next' in the RvizVisualToolsGui to continue");
}


FindGraspPose::CandidateGrasp FindGraspPose::sampleCandidateGrasp(
  const pcl::PointCloud<pcl::PointNormal>::ConstPtr & cloud,
  const pcl::PointCloud<pcl::PrincipalCurvatures>::ConstPtr & pcs,
  bool interactive)
{
  CandidateGrasp result;

  if (interactive) {
    visual_tools_->deleteAllMarkers();
  }

  // Generate random test point
  const size_t idx = std::rand() % cloud->size();
  result.selected_point_index = idx;

  /* Find Darboux frame
   * x axis: direction of maximum curvature (principal curvature)
   * y axis: surface normal pointing inwards
   * z axis: minor axis of curvature (cross product of x and y)
   */
  const auto & pt = cloud->points[idx];
  Eigen::Vector3d p(pt.x, pt.y, pt.z);

  const auto & curvature = pcs->points[idx];
  Eigen::Vector3d n(pt.normal_x, pt.normal_y, pt.normal_z);
  n = -n.normalized();  // Unit normal pointing inwards
  Eigen::Vector3d d1(
    curvature.principal_curvature_x,
    curvature.principal_curvature_y,
    curvature.principal_curvature_z);
  d1.normalize();

  Eigen::Vector3d d2 = d1.cross(n);
  d2.normalize();  // TODO should already be unit length
  n = d2.cross(d1);  // re-orthogonalize n  TODO should already be orthonormal
  Eigen::Matrix3d R;
  R.col(0) = d1;
  R.col(1) = n;
  R.col(2) = d2;

  // Build Darboux frame
  Eigen::Isometry3d T_dbx = Eigen::Isometry3d::Identity();
  T_dbx.translation() = p;
  T_dbx.linear() = R;
  if (interactive) {
    visual_tools_->publishAxisLabeled(T_dbx, "Darboux_frame", rviz_visual_tools::Scales::SMALL);
    visual_tools_->trigger();
  }

  // Get grasp→gripper_base transform (only once)
  static const std::string grasp_link = "grasp_link";
  static const std::string gripper_base_link = "robotiq_85_base_link";
  if (!T_gripper_grasp_) {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        grasp_link, gripper_base_link, rclcpp::Time(0));
      // This gives T_gripper_base_grasp (grasp → gripper_base)
      T_gripper_grasp_ = std::make_shared<Eigen::Isometry3d>(
        tf2::transformToEigen(transform.transform));
      RCLCPP_INFO(
        node_->get_logger(), "Got transform %s -> %s: %.3f %.3f %.3f",
        grasp_link.c_str(), gripper_base_link.c_str(),
        T_gripper_grasp_->translation().x(),
        T_gripper_grasp_->translation().y(),
        T_gripper_grasp_->translation().z());  // TODO remove after checking it's not inverted
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        node_->get_logger(), "Could not get transform %s -> %s: %s",
        grasp_link.c_str(), gripper_base_link.c_str(), ex.what());
      result.success = false;
      return result;
    }
  }

  // Collision line search along Darboux frame y axis
  const double search_limit = 0.05;  // m, +-
  const size_t n_steps = 10;         // total

  std::unique_ptr<Eigen::Isometry3d> T_gripper_base_candidate_best;
  for (size_t i = 0; i < n_steps; i++) {
    // Starts from y = -search_limit, because dbx y axis points inwards
    double offset = (static_cast<double>(i) / n_steps - 0.5) * 2.0 * search_limit;

    /* Rotate Dbx to Gripper orientation
     *
     * x axes align
     *
     * Dbx:
     * - x points towards greatest curvature
     * - y points inwards
     * - z points along smallest curvature
     *
     * Gripper:
     * - x points along axis where fingers move
     * - y points perp to approach
     * - z points inwards
     *
     * We have to rotate Dbx -90° around its local x axis
     */
    Eigen::Isometry3d T_grasp_offset = T_dbx;

    // Rotate around local x axis of T_dbx
    Eigen::Matrix3d R_gripper_from_world;
    Eigen::Matrix3d R_dbx = T_dbx.linear();
    R_gripper_from_world.col(0) = R_dbx.col(0);        // Gripper.x  = Dbx.x
    R_gripper_from_world.col(2) = R_dbx.col(1);        // Gripper.z  = Dbx.y
    R_gripper_from_world.col(1) = -R_dbx.col(2);       // Gripper.y  = -Dbx.z
    T_grasp_offset.linear() = R_gripper_from_world;

    // Apply offset along local z axis (after rotation)
    T_grasp_offset.translation() += offset * T_grasp_offset.linear().col(2);

    // Compose correctly: world→gripper_base = world→grasp * grasp→gripper_base
    const Eigen::Isometry3d T_gripper_base_candidate =
      T_grasp_offset * (*T_gripper_grasp_);

    // Check collision
    bool in_collision = true;
    bool feasible = true;
    const auto kinematic_state = getRobotStateForPose(T_gripper_base_candidate, 0.0);
    try {
      in_collision = checkGripperCollision(kinematic_state);
    } catch (const std::runtime_error & e) {
      RCLCPP_DEBUG(node_->get_logger(), "Collision check failed: %s", e.what());
      feasible = false;
    }

    if (interactive) {
      std::string status;
      rviz_visual_tools::Colors color;
      if (!feasible) {
        status = "ik failed";
        color = rviz_visual_tools::Colors::BLUE;
      } else if (in_collision) {
        status = "in collision";
        color = rviz_visual_tools::Colors::RED;
      } else {
        status = "collision-free";
        color = rviz_visual_tools::Colors::GREEN;
      }

      if (interactive) {
        // Only visualize collision-free gripper to avoid clutter
        if (color == rviz_visual_tools::Colors::GREEN) {
          visualizeGripper(kinematic_state, color);
        }
      }

      RCLCPP_DEBUG(node_->get_logger(), "Grasp pose (offset %.3f): %s", offset, status.c_str());
    }

    if (feasible && !in_collision)
      T_gripper_base_candidate_best = std::make_unique<Eigen::Isometry3d>(
        T_gripper_base_candidate);
  }

  if (!T_gripper_base_candidate_best) {
    RCLCPP_WARN(node_->get_logger(), "No feasible grasp pose found");
    result.success = false;
    return result;
  }

  if (interactive) {
    // visual_tools_->prompt("Press 'next' in the RvizVisualToolsGui to continue");
    // visual_tools_->deleteAllMarkers();

    const auto kinematic_state = getRobotStateForPose(*T_gripper_base_candidate_best, 0.0);
    visualizeGripper(
      kinematic_state, rviz_visual_tools::Colors::GREEN);
  }

  // Set cost to gripper y axis alignment with world negative z axis (pointing down)
  static const double cost_weight_z = 1.0;  // TODO set via param
  const double cost_gripper_z = cost_weight_z *
    (1.0 - T_gripper_base_candidate_best->linear().col(1).dot(Eigen::Vector3d::UnitZ()));

  // Evaluate inliers: points between the gripper fingers
  const auto& T_grasp_world = *T_gripper_base_candidate_best * T_gripper_grasp_->inverse();
  static const double gripper_inner_width = 0.085;
  static const double finger_length = 0.038;
  static const double finger_width = 0.022;
  if (interactive) {
    visual_tools_->publishWireframeCuboid(
      T_grasp_world,
      gripper_inner_width, finger_width, finger_length,
      rviz_visual_tools::Colors::YELLOW);
  }

  // Gripper axes in world frame
  Eigen::Vector3d gripper_x = T_grasp_world.linear().col(0);  // closing direction
  Eigen::Vector3d gripper_y = T_grasp_world.linear().col(1);  // lateral
  Eigen::Vector3d gripper_z = T_grasp_world.linear().col(2);  // approach
  Eigen::Vector3d p_center = T_grasp_world.translation();

  const double hx = gripper_inner_width / 2.0;
  const double hy = finger_width / 2.0;
  const double hz = finger_length / 2.0;

  // Loop through points
  std::vector<pcl::PointNormal> inlier_points;
  for (const auto& pt : cloud->points) {
    Eigen::Vector3d p(pt.x, pt.y, pt.z);
    Eigen::Vector3d diff = p - p_center;

    // Project onto gripper axes
    double dx = diff.dot(gripper_x);
    double dy = diff.dot(gripper_y);
    double dz = diff.dot(gripper_z);

    // Check bounds in gripper-aligned coordinates
    if (std::abs(dx) <= hx &&
        std::abs(dy) <= hy &&
        std::abs(dz) <= hz) {
      inlier_points.push_back(pt);
    }
  }

  result.inliers = inlier_points.size();
  static const size_t min_inliers = 5;  // TODO set via param
  if (result.inliers < min_inliers) {
    RCLCPP_WARN(
      node_->get_logger(), "Not enough inliers (%zu < %zu)", result.inliers, min_inliers);
    result.success = false;
    return result;
  }

  // Calculate cost of point normals alignment with gripper closing direction (x axis)
  double cost_normals = 0.0;
  for (const auto& pt : inlier_points) {
    Eigen::Vector3d p(pt.x, pt.y, pt.z);
    Eigen::Vector3d n(pt.normal_x, pt.normal_y, pt.normal_z);

    cost_normals -= std::pow(n.dot(gripper_x), 2);

    // Visualize
    if (interactive) {
      Eigen::Vector3d p_end = p + 0.01 * n;
      geometry_msgs::msg::Point p_start_msg = tf2::toMsg(p);
      geometry_msgs::msg::Point p_end_msg = tf2::toMsg(p_end);

      rviz_visual_tools::Colors color;
      if (std::abs(n.dot(gripper_x)) > 0.866)  // cos(30°)
        color = rviz_visual_tools::Colors::GREEN;
      else if (std::abs(n.dot(gripper_x)) > 0.5)  // cos(60°)
        color = rviz_visual_tools::Colors::YELLOW;
      else
        color = rviz_visual_tools::Colors::RED;
      visual_tools_->publishArrow(p_start_msg, p_end_msg, color, rviz_visual_tools::Scales::XXSMALL);
    }
  }

  cost_normals /= static_cast<double>(inlier_points.size());
  result.cost = cost_gripper_z + cost_normals;

  RCLCPP_INFO(
    node_->get_logger(), "Candidate grasp: cost: %.3f (z: %.3f, normals: %.3f), inliers: %zu",
    result.cost, cost_gripper_z, cost_normals, result.inliers);

  if (interactive) {
    visual_tools_->trigger();
    visual_tools_->prompt("Press 'next' in the RvizVisualToolsGui to continue");
  }

  result.pose = *T_gripper_base_candidate_best;
  result.success = true;
  return result;
}

void FindGraspPose::visualizeGripper(
  const moveit::core::RobotStatePtr kinematic_state,
  rviz_visual_tools::Colors color)
{
  if (!robot_model_loader_) {
    throw std::runtime_error("Robot model loader not initialized");
  }

  auto robot_model = robot_model_loader_->getModel();
  if (!robot_model) {
    throw std::runtime_error("Robot model not available");
  }

  if (!kinematic_state) {
    RCLCPP_ERROR(node_->get_logger(), "Kinematic state is null, cannot visualize gripper");
    return;
  }

  // Compute transform from robot base to gripper root link
  const std::string gripper_root_link = "robotiq_85_base_link";
  Eigen::Isometry3d base_pose = Eigen::Isometry3d::Identity();
  base_pose = kinematic_state->getGlobalLinkTransform(gripper_root_link);

  const auto * root_link_model = robot_model->getLinkModel(gripper_root_link);
  if (!root_link_model) {
    throw std::runtime_error("Gripper root link model not found");
  }

  // Get current root link transform in the robot’s base frame
  Eigen::Isometry3d T_base_root = kinematic_state->getGlobalLinkTransform(gripper_root_link);

  // Compute offset so that "base_pose" becomes the new root
  // i.e. we want: world_tf = base_pose * (T_base_root^-1 * T_base_link)
  Eigen::Isometry3d T_root_base = T_base_root.inverse();

  // recursive lambda to visualize link subtree (via child joints)
  std::function<void(const moveit::core::LinkModel*)> visualizeLinkRecursive;
  visualizeLinkRecursive = [&](const moveit::core::LinkModel * link_model)
  {
    if (!link_model) return;

    const std::string link_name = link_model->getName();
    const std::string & mesh_filename = link_model->getVisualMeshFilename();
    const Eigen::Isometry3d & mesh_origin = link_model->getVisualMeshOrigin();

    // Transform link w.r.t. root
    Eigen::Isometry3d T_base_link = kinematic_state->getGlobalLinkTransform(link_name);
    Eigen::Isometry3d T_root_link = T_root_base * T_base_link;

    // Apply user-specified base pose
    Eigen::Isometry3d world_tf = base_pose * T_root_link;

    if (!mesh_filename.empty()) {
      Eigen::Isometry3d final_tf = world_tf * mesh_origin;
      visual_tools_->publishMesh(final_tf, mesh_filename, color);
    }

    for (const auto * child_joint : link_model->getChildJointModels()) {
      if (!child_joint) continue;
      visualizeLinkRecursive(child_joint->getChildLinkModel());
    }
  };

  visualizeLinkRecursive(root_link_model);
  visual_tools_->trigger();
}

pcl::PointCloud<pcl::PointNormal>::Ptr FindGraspPose::computeNormals(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input,
  const geometry_msgs::msg::Point & viewpoint,
  const int k_neighbors)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(input);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  ne.setKSearch(k_neighbors);
  ne.setViewPoint(viewpoint.x, viewpoint.y, viewpoint.z);

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
  ne.compute(*normals);

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
  pcl::concatenateFields(*input, *normals, *cloud_with_normals);
  return cloud_with_normals;
}

pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr FindGraspPose::computePrincipalCurvatures(
  const pcl::PointCloud<pcl::PointNormal>::ConstPtr & input,
  const int k_neighbors)
{
  pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pce;

  // Separate points and normals
  pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
  pcl::copyPointCloud(*input, *points);
  pcl::copyPointCloud(*input, *normals);
  pce.setInputCloud(points);
  pce.setInputNormals(normals);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  pce.setSearchMethod(tree);
  pce.setKSearch(k_neighbors);

  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr pcs(new pcl::PointCloud<pcl::PrincipalCurvatures>());
  pce.compute(*pcs);

  return pcs;
}

bool FindGraspPose::checkGripperCollision(
  const moveit::core::RobotStatePtr kinematic_state)
{
  // Lock planning scene
  if (!psm_) {
    throw std::runtime_error("PlanningSceneMonitor not initialized.");
  }

  if (!kinematic_state) {
    RCLCPP_ERROR(node_->get_logger(), "Kinematic state is null, cannot check collision.");
    return true;
  }

  planning_scene_monitor::LockedPlanningSceneRO scene_lock(psm_);
  const planning_scene::PlanningSceneConstPtr& scene = scene_lock.operator->();
  if (!scene) {
    throw std::runtime_error("Planning scene not available.");
  }

  // Collision checking
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_result.clear();

  // collision_request.group_name = move_group_gripper_->getName();
  // collision_request.contacts = true;
  // collision_request.max_contacts = 100;
  // collision_request.verbose = false;

  scene->checkCollision(collision_request, collision_result, *kinematic_state);
  return collision_result.collision;
}

moveit::core::RobotStatePtr FindGraspPose::getRobotStateForPose(
  const Eigen::Isometry3d& gripper_base_pose,
  double gripper_joint_value)
{
  if (!visual_tools_ || !move_group_arm_) {
    throw std::runtime_error("Visual tools or move group not initialized.");
  }

  // Get a copy of the current robot state
  moveit::core::RobotStatePtr current_state = move_group_arm_->getCurrentState();
  if (!current_state) {
    throw std::runtime_error("Current robot state not available.");
  }

  auto kinematic_state = std::make_shared<moveit::core::RobotState>(*current_state);

  // Solve IK for gripper base pose
  const std::string gripper_base_link = "robotiq_85_base_link";
  bool found_ik = kinematic_state->setFromIK(
    move_group_arm_->getCurrentState()->getJointModelGroup(move_group_arm_->getName()),
    gripper_base_pose, gripper_base_link, 0.1);
  if (!found_ik) {
    RCLCPP_ERROR(
      node_->get_logger(), "IK solution not found for gripper base pose");
    return nullptr;
  }

  // Set desired gripper joint value(s). Here we set only "gripper_joint".
  // If you want to control multiple gripper joints, accept a map<string,double> instead.
  const std::string gripper_joint_name = "robotiq_85_left_knuckle_joint";

  // For single-DOF joint: TODO check joint type
  kinematic_state->setJointPositions(gripper_joint_name, &gripper_joint_value);

  // IMPORTANT: update transforms before querying them
  kinematic_state->update();  // ensures checkLinkTransforms() will pass

  return kinematic_state;
}

}  // namespace bin_picking


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto bin_picking = std::make_shared<bin_picking::FindGraspPose>();
  bin_picking->run();

  rclcpp::shutdown();
  return 0;
}
