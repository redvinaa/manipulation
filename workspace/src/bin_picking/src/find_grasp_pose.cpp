#include <thread>

#include "bin_picking/find_grasp_pose.hpp"
#include "bin_picking/utils.hpp"

#include <rclcpp/executors.hpp>
#include <rclcpp/callback_group.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>  // for concatenatePointCloud

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>


namespace bin_picking
{

using namespace std::chrono_literals;

FindGraspPose::FindGraspPose()
{
  // Create ROS2 node
  node_ = rclcpp::Node::make_shared("find_grasp_pose");
  mgi_node_ = rclcpp::Node::make_shared("find_grasp_pose_mgi");
  vis_node_ = rclcpp::Node::make_shared("find_grasp_pose_vis");
  executor_.add_node(node_);
  executor_.add_node(mgi_node_);
  executor_.add_node(vis_node_);
  spin_thread_ = std::thread([this]() { executor_.spin(); });

  // Declare parameter for pointcloud topics
  std::vector<std::string> default_topics;
  node_->declare_parameter("pointcloud_topics", default_topics);
  pointcloud_topics_ = node_->get_parameter("pointcloud_topics").as_string_array();

  node_->declare_parameter<std::string>("target_frame", "world");
  target_frame_ = node_->get_parameter("target_frame").as_string();

  node_->declare_parameter<float>("voxel_size", 0.01f);
  voxel_size_ = node_->get_parameter("voxel_size").as_double();

  node_->declare_parameter<std::string>("robot_description", "");

  node_->declare_parameter<float>("min_x", -std::numeric_limits<float>::max());
  min_x_ = node_->get_parameter("min_x").as_double();
  node_->declare_parameter<float>("max_x", std::numeric_limits<float>::max());
  max_x_ = node_->get_parameter("max_x").as_double();
  node_->declare_parameter<float>("min_y", -std::numeric_limits<float>::max());
  min_y_ = node_->get_parameter("min_y").as_double();
  node_->declare_parameter<float>("max_y", std::numeric_limits<float>::max());
  max_y_ = node_->get_parameter("max_y").as_double();
  node_->declare_parameter<float>("min_z", -std::numeric_limits<float>::max());
  min_z_ = node_->get_parameter("min_z").as_double();
  node_->declare_parameter<float>("max_z", std::numeric_limits<float>::max());
  max_z_ = node_->get_parameter("max_z").as_double();

  if (pointcloud_topics_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No pointcloud topics configured!");
  }

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

  // Initialize PlanningSceneMonitor
  psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
    node_, "robot_description");
  psm_->startSceneMonitor("/monitored_planning_scene");

  // // Test visualizeGripper
  // // Move in a sin wave between y = +-0.5
  // const float d_angle = 2 * M_PI * 0.1 * 0.1; // 0.1 Hz
  // const float amplitude = 0.3;
  // Eigen::Isometry3d gripper_pose = Eigen::Isometry3d::Identity();
  // gripper_pose.translation() = Eigen::Vector3d(0.5, 0.0, 0.25);  // fixed x and z
  // gripper_pose.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix(); // point downwards
  // double angle = 0;

  // while (true) {
  //   visual_tools_->deleteAllMarkers();

  //   gripper_pose.translation().y() = amplitude * std::sin(angle);

  //   rviz_visual_tools::Colors color = rviz_visual_tools::BLUE;  // default for unfeasible poses

  //   try {
  //     const bool in_collision = checkCollision(gripper_pose);
  //     color = in_collision ? rviz_visual_tools::RED : rviz_visual_tools::GREEN;
  //   } catch (const std::runtime_error & e) {
  //   }

  //   visualizeGripper(gripper_pose, 0.0, color);

  //   std::this_thread::sleep_for(100ms);
  //   angle += 0.1;
  //   angle = std::fmod(angle, 2 * M_PI);
  // }

  // Create a subscriber for each topic
  for (const auto & topic : pointcloud_topics_) {
    auto sub = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic,
      rclcpp::SensorDataQoS(),
      [this, topic](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        pointCloudCallback(msg, topic);
      });
    subscriptions_.push_back(sub);
    RCLCPP_INFO(node_->get_logger(), "Subscribed to %s", topic.c_str());
  }

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
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,
  const std::string & topic_name)
{
  MeasureExecutionTimeScoped timer("pointCloudCallback");

  // Convert ROS msg -> PCL cloud
  MeasureExecutionTime timer_conv("fromROSMsg");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *cloud);
  timer_conv.stop();

  RCLCPP_INFO(node_->get_logger(), "==== Received cloud from %s with %zu points ====",
              topic_name.c_str(), cloud->size());

  MeasureExecutionTime timer_transform("transformPointCloud");
  cloud = transformPointCloud(cloud, target_frame_);
  timer_transform.stop();

  // Crop to region of interest
  MeasureExecutionTime timer_crop("cropPointCloud");
  cloud = cropPointCloud(cloud, min_x_, max_x_, min_y_, max_y_, min_z_, max_z_);
  timer_crop.stop();

  // Compute normals (optional, may be used later)
  MeasureExecutionTime timer_normals("computeNormals");
  geometry_msgs::msg::Point viewpoint;
  try {
    auto transform_stamped = tf_buffer_->lookupTransform(
        target_frame_, msg->header.frame_id, rclcpp::Time(0));
    viewpoint.x = transform_stamped.transform.translation.x;
    viewpoint.y = transform_stamped.transform.translation.y;
    viewpoint.z = transform_stamped.transform.translation.z;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "Could not get camera_link position: %s", ex.what());
    viewpoint.x = viewpoint.y = viewpoint.z = 0.0;
  }
  auto with_normals = computeNormals(cloud, viewpoint);
  timer_normals.stop();

  // Store voxelized cloud
  clouds_with_normals_[topic_name] = with_normals;

  // Visualize (optional, here just show cloud size in console)
  RCLCPP_INFO(node_->get_logger(),
              "Processed voxelized cloud from %s with %zu points",
              topic_name.c_str(), with_normals->size());

  // If we have received clouds from all topics, merge them
  if (clouds_with_normals_.size() == pointcloud_topics_.size()) {
    std::vector<pcl::PointCloud<pcl::PointNormal>::ConstPtr> cloud_vec;
    cloud_vec.reserve(clouds_with_normals_.size());
    for (auto &kv : clouds_with_normals_) {
      cloud_vec.push_back(kv.second);
    }

    MeasureExecutionTime timer_merge("mergeClouds");
    const auto merged_cloud = mergeClouds(cloud_vec, voxel_size_);
    timer_merge.stop();

    // Clear stored clouds for next round
    clouds_with_normals_.clear();

    // Visualize normals as lines
    visual_tools_->deleteAllMarkers();
    for (size_t i = 0; i < merged_cloud->size(); i += 10) {  // visualize every 10th normal
      const auto & pt = merged_cloud->points[i];
      if (!std::isnan(pt.normal_x) && !std::isnan(pt.normal_y) && !std::isnan(pt.normal_z)) {
        Eigen::Vector3d start(pt.x, pt.y, pt.z);
        Eigen::Vector3d end(pt.x + 0.01 * pt.normal_x,
                            pt.y + 0.01 * pt.normal_y,
                            pt.z + 0.01 * pt.normal_z);

        // Color based on curvature
        rviz_visual_tools::Colors color;
        if (pt.curvature < 0.01)
          color = rviz_visual_tools::GREEN;
        else if (pt.curvature < 0.03)
          color = rviz_visual_tools::YELLOW;
        else
          color = rviz_visual_tools::RED;

        visual_tools_->publishLine(
          start, end, color, rviz_visual_tools::Scales::XXXXSMALL);
      }
    }
    visual_tools_->trigger();

    // Compute principal curvatures
    MeasureExecutionTime timer_pcs("computePrincipalCurvatures");
    auto pcs = computePrincipalCurvatures(merged_cloud);
    timer_pcs.stop();

    // Generate random test point
    const size_t idx = std::rand() % merged_cloud->size();

    /* Find Darboux frame
     * x axis: direction of maximum curvature (principal curvature)
     * y axis: surface normal pointing inwards
     * z axis: minor axis of curvature (cross product of x and y)
     */
    const auto & pt = merged_cloud->points[idx];
    Eigen::Vector3d p(pt.x, pt.y, pt.z);

    const auto & curvature = pcs->points[idx];
    Eigen::Vector3d n(pt.normal_x, pt.normal_y, pt.normal_z);
    n = -n / n.norm(); // Unit normal pointing inwards
    Eigen::Vector3d d1(curvature.principal_curvature_x, curvature.principal_curvature_y, curvature.principal_curvature_z);
    d1.normalize();

    Eigen::Vector3d d2 = d1.cross(n);
    // d2.normalize();  // already unit length
    Eigen::Matrix3d R;
    R.col(0) = d1;
    R.col(1) = n;
    R.col(2) = d2;

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = p;
    T.linear() = R;
    visual_tools_->publishAxisLabeled(T.cast<double>(), "Darboux_frame");
    visual_tools_->trigger();

    // Distance offset along surface normal
    const double distance_to_surface = 0.01;
    Eigen::Vector3d p_grasp = p - distance_to_surface * n;

    // Build Darboux frame
    Eigen::Isometry3d T_grasp = Eigen::Isometry3d::Identity();
    T_grasp.translation() = p_grasp;
    T_grasp.linear() = R;

    // ------------------------------------------------------------------
    // Adjust: express T_grasp at gripper root (e.g. wrist_3_link) instead of grasp_link
    // ------------------------------------------------------------------
    static robot_model_loader::RobotModelLoader loader(node_, "robot_description");
    auto robot_model = loader.getModel();
    moveit::core::RobotStatePtr state_ptr = move_group_arm_->getCurrentState();
    moveit::core::RobotState robot_state = *state_ptr;

    // Define links
    const std::string grasp_link = "grasp_link";
    const std::string root_link  = "robotiq_85_base_link";

    // Get transforms from robot_state (both in base frame)
    Eigen::Isometry3d T_base_grasp = robot_state.getGlobalLinkTransform(grasp_link);
    Eigen::Isometry3d T_base_root  = robot_state.getGlobalLinkTransform(root_link);

    // Compute fixed transform root←grasp
    Eigen::Isometry3d T_root_grasp = T_base_root.inverse() * T_base_grasp;

    // Final pose: where the root link should be, given the Darboux frame at grasp_link
    Eigen::Isometry3d T_root = T_grasp * T_root_grasp.inverse();

    // ------------------------------------------------------------------
    // Collision check now at root pose
    // ------------------------------------------------------------------
    rviz_visual_tools::Colors color = rviz_visual_tools::BLUE;  // default for unfeasible poses
    try {
      const bool in_collision = checkCollision(T_root);
      RCLCPP_INFO(node_->get_logger(), "Grasp pose is %s",
                  in_collision ? "in collision" : "collision-free");
      color = in_collision ? rviz_visual_tools::RED : rviz_visual_tools::GREEN;
    } catch (const std::runtime_error & e) {
      // RCLCPP_WARN(node_->get_logger(), "Collision check failed: %s", e.what());
    }

    visualizeGripper(T_root, 0.0, color);

    visual_tools_->prompt("Press 'next' in the RvizVisualToolsGui to continue");

    // Linearly search for a valid grasp pose
    // TODO
  }
}

std::vector<int> FindGraspPose::visualizeGripper(
  const Eigen::Isometry3d & base_pose,
  double gripper_joint_value,
  rviz_visual_tools::Colors color)
{
  if (!visual_tools_ || !move_group_arm_) {
    RCLCPP_WARN(node_->get_logger(), "Visual tools or move group not initialized.");
    return {};
  }

  // Load robot model once (static)
  // TODO create overload that accepts a RobotModelPtr
  static robot_model_loader::RobotModelLoader loader(node_, "robot_description");
  auto robot_model = loader.getModel();
  if (!robot_model) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load robot model");
    return {};
  }

  // Get the gripper group's root link name
  const std::string gripper_group_name = "gripper";
  const auto * gripper_group = robot_model->getJointModelGroup(gripper_group_name);
  if (!gripper_group) {
    RCLCPP_ERROR(node_->get_logger(), "Could not find joint group '%s'", gripper_group_name.c_str());
    return {};
  }

  // Get a fresh RobotState from MoveGroup (this has valid transforms)
  moveit::core::RobotStatePtr current_state_ptr = move_group_arm_->getCurrentState();
  if (!current_state_ptr) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get current robot state from move_group_arm_");
    return {};
  }
  // Copy to local state so we can modify without affecting MoveGroup internals
  moveit::core::RobotState robot_state = *current_state_ptr;

  // Set desired gripper joint value(s). Here we set only "gripper_joint".
  // If you want to control multiple gripper joints, accept a map<string,double> instead.
  const std::string gripper_joint_name = "robotiq_85_left_knuckle_joint";

  // For single-DOF joint: TODO check joint type
  robot_state.setJointPositions(gripper_joint_name, &gripper_joint_value);

  // IMPORTANT: update transforms before querying them
  robot_state.update();  // ensures checkLinkTransforms() will pass

  std::vector<int> marker_ids;

  // Compute transform from robot base to gripper root link
  const std::string root_link = "robotiq_85_base_link";
  const auto * root_link_model = robot_model->getLinkModel(root_link);
  if (!root_link_model) {
    RCLCPP_ERROR(node_->get_logger(), "Could not find link model for root link '%s'", root_link.c_str());
    return {};
  }

  // Get current root link transform in the robot’s base frame
  Eigen::Isometry3d T_base_root = robot_state.getGlobalLinkTransform(root_link);

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
    Eigen::Isometry3d T_base_link = robot_state.getGlobalLinkTransform(link_name);
    Eigen::Isometry3d T_root_link = T_root_base * T_base_link;

    // Apply user-specified base pose
    Eigen::Isometry3d world_tf = base_pose * T_root_link;

    if (!mesh_filename.empty()) {
      Eigen::Isometry3d final_tf = world_tf * mesh_origin;
      visual_tools_->publishMesh(final_tf, mesh_filename, color);
      int id = visual_tools_->getMeshId();
      marker_ids.push_back(id);
    }

    for (const auto * child_joint : link_model->getChildJointModels()) {
      if (!child_joint) continue;
      visualizeLinkRecursive(child_joint->getChildLinkModel());
    }
  };

  visualizeLinkRecursive(root_link_model);
  visual_tools_->trigger();

  return marker_ids;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FindGraspPose::transformPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud,
    const std::string & target_frame)
{
  // Lookup the transform from cloud frame to target frame
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
      transform_stamped = tf_buffer_->lookupTransform(
          target_frame, cloud->header.frame_id,
          rclcpp::Time(cloud->header.stamp));
  } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(node_->get_logger(), "Could not transform point cloud: %s", ex.what());
      return nullptr;
  }

  // Convert geometry_msgs Transform to Eigen Affine3f
  Eigen::Affine3f transform_eigen = Eigen::Affine3f::Identity();
  transform_eigen.translation() <<
      transform_stamped.transform.translation.x,
      transform_stamped.transform.translation.y,
      transform_stamped.transform.translation.z;
  transform_eigen.linear() = Eigen::Quaternionf(
      transform_stamped.transform.rotation.w,
      transform_stamped.transform.rotation.x,
      transform_stamped.transform.rotation.y,
      transform_stamped.transform.rotation.z).toRotationMatrix();

  // Execute the transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*cloud, *transformed_cloud, transform_eigen);

  return transformed_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FindGraspPose::cropPointCloud(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input,
  float min_x, float max_x,
  float min_y, float max_y,
  float min_z, float max_z)
{
    // Initialize the CropBox filter
    pcl::CropBox<pcl::PointXYZ> crop_filter;
    crop_filter.setInputCloud(input);

    // Define the minimum and maximum points of the box
    Eigen::Vector4f min_pt(min_x, min_y, min_z, 1.0);
    Eigen::Vector4f max_pt(max_x, max_y, max_z, 1.0);

    // Set the crop box parameters
    crop_filter.setMin(min_pt);
    crop_filter.setMax(max_pt);

    // Apply the filter and store the result
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    crop_filter.filter(*cropped_cloud);

    return cropped_cloud;
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

pcl::PointCloud<pcl::PointNormal>::Ptr FindGraspPose::mergeClouds(
  const std::vector<pcl::PointCloud<pcl::PointNormal>::ConstPtr> & clouds,
  const float voxel_size)
{
  // Concatenate all point clouds into one
  pcl::PointCloud<pcl::PointNormal>::Ptr merged(new pcl::PointCloud<pcl::PointNormal>());
  for (const auto &cloud : clouds) {
    if (cloud && !cloud->empty()) {
      *merged += *cloud;
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Merged cloud contains %zu points before downsampling", merged->size());

  // Apply VoxelGrid filter to downsample
  pcl::VoxelGrid<pcl::PointNormal> voxel_filter;
  voxel_filter.setInputCloud(merged);
  voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);  // cubic voxel grid :contentReference[oaicite:1]{index=1}

  pcl::PointCloud<pcl::PointNormal>::Ptr downsampled(new pcl::PointCloud<pcl::PointNormal>());
  voxel_filter.filter(*downsampled);

  RCLCPP_INFO(
    node_->get_logger(), "Downsampled cloud contains %zu points after voxel grid size %.3f",
    downsampled->size(), voxel_size);

  return downsampled;
}

bool FindGraspPose::checkCollision(
  const Eigen::Isometry3d & gripper_base_pose, double gripper_joint)
{
  // // --- Step 1: Transform grasp_link pose to gripper_link pose ---
  // // Example: grasp_link is at finger tips center, gripper_link is base of fingers
  // Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  // offset.translation() = Eigen::Vector3d(0.0, 0.0, -0.14); // 5 cm back along Z

  // Eigen::Isometry3d gripper_pose = grasp_pose * offset.inverse();

  Eigen::Isometry3d gripper_pose = gripper_base_pose;

  // --- Step 2: Solve IK for arm ---
  moveit::core::RobotStatePtr kinematic_state = move_group_arm_->getCurrentState();
  if (!kinematic_state) {
    throw std::runtime_error("Current robot state not available.");
  }

  const moveit::core::JointModelGroup* arm_jmg =
    kinematic_state->getJointModelGroup(move_group_arm_->getName());

  bool found_ik = kinematic_state->setFromIK(arm_jmg, gripper_pose);
  if (!found_ik) {
    throw std::runtime_error("IK solution not found for given gripper pose.");
  }

  // --- Step 3: Set gripper joint(s) ---
  std::vector<std::string> gripper_joints = move_group_gripper_->getJoints();
  std::vector<double> gripper_values(gripper_joints.size(), gripper_joint);
  kinematic_state->setJointGroupPositions(move_group_gripper_->getName(), gripper_values);

  // --- Step 4: Check collision ---
  planning_scene_monitor::LockedPlanningSceneRO planning_scene(psm_);
  if (!planning_scene) {
    throw std::runtime_error("Planning scene not available.");
  }

  collision_detection::CollisionRequest collision_request;
  collision_request.group_name = move_group_gripper_->getName(); // only check gripper
  collision_request.verbose = true;
  collision_detection::CollisionResult collision_result;

  planning_scene->checkCollision(collision_request, collision_result, *kinematic_state);

  if (collision_result.collision) {
    return true;
  }
  else {
    return false;
  }
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
