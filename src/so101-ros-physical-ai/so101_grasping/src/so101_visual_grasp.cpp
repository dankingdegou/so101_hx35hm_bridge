#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/parallel_gripper_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <deque>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <thread>
#include <algorithm>
#include <vector>

using GripperCommand = control_msgs::action::ParallelGripperCommand;

static double median_of(std::vector<double> values)
{
  if (values.empty()) return 0.0;
  const auto mid = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), mid, values.end());
  if ((values.size() % 2U) == 1U) return *mid;
  const auto lower_mid = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2 - 1);
  std::nth_element(values.begin(), lower_mid, values.end());
  return 0.5 * (*mid + *lower_mid);
}

static bool ensure_robot_model_params_from_move_group(
  const rclcpp::Node::SharedPtr& node,
  const std::string& move_group_node)
{
  if (!node) return false;

  try {
    const auto urdf = node->get_parameter("robot_description").as_string();
    const auto srdf = node->get_parameter("robot_description_semantic").as_string();
    if (!urdf.empty() && !srdf.empty()) return true;
  } catch (...) {
  }

  auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, move_group_node);
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(
      node->get_logger(),
      "move_group parameters service not available at '%s'. Start MoveIt first (move_group).",
      move_group_node.c_str());
    return false;
  }

  const std::vector<std::string> keys = {"robot_description", "robot_description_semantic"};
  auto fut = client->get_parameters(keys);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  const auto rc = exec.spin_until_future_complete(fut, std::chrono::seconds(5));
  exec.remove_node(node);

  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Timed out fetching robot_description from '%s'.", move_group_node.c_str());
    return false;
  }

  const auto params = fut.get();
  if (params.size() != keys.size()) {
    RCLCPP_ERROR(node->get_logger(), "Unexpected parameter list size from move_group.");
    return false;
  }

  const auto urdf2 = params[0].as_string();
  const auto srdf2 = params[1].as_string();
  if (urdf2.empty() || srdf2.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Fetched empty robot model params from '%s' (urdf=%zu bytes, srdf=%zu bytes).",
      move_group_node.c_str(), urdf2.size(), srdf2.size());
    return false;
  }

  node->set_parameter(rclcpp::Parameter("robot_description", urdf2));
  node->set_parameter(rclcpp::Parameter("robot_description_semantic", srdf2));
  RCLCPP_INFO(
    node->get_logger(),
    "Loaded robot model params from '%s' (urdf=%zu bytes, srdf=%zu bytes).",
    move_group_node.c_str(), urdf2.size(), srdf2.size());

  auto copy_ns = [&](const std::string& prefix) {
    auto list_fut = client->list_parameters({prefix}, 10);
    rclcpp::executors::SingleThreadedExecutor e;
    e.add_node(node);
    const auto lrc = e.spin_until_future_complete(list_fut, std::chrono::seconds(5));
    e.remove_node(node);
    if (lrc != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(node->get_logger(), "Timed out listing parameters under '%s' from '%s'.",
                  prefix.c_str(), move_group_node.c_str());
      return;
    }
    const auto result = list_fut.get();
    if (result.names.empty()) return;

    auto get_fut = client->get_parameters(result.names);
    rclcpp::executors::SingleThreadedExecutor e2;
    e2.add_node(node);
    const auto grc = e2.spin_until_future_complete(get_fut, std::chrono::seconds(5));
    e2.remove_node(node);
    if (grc != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(node->get_logger(), "Timed out getting parameters under '%s' from '%s'.",
                  prefix.c_str(), move_group_node.c_str());
      return;
    }
    for (const auto& p : get_fut.get()) {
      node->set_parameter(p);
    }
    RCLCPP_INFO(node->get_logger(), "Copied %zu parameters under '%s' from '%s'.",
                result.names.size(), prefix.c_str(), move_group_node.c_str());
  };

  copy_ns("robot_description_kinematics");
  copy_ns("robot_description_planning");

  return true;
}

static bool send_gripper_command(
  const rclcpp::Node::SharedPtr& node,
  const std::string& action_name,
  double position,
  double max_effort,
  double timeout_s)
{
  if (!node) return false;
  rclcpp::NodeOptions helper_options;
  helper_options.use_global_arguments(false);
  auto helper = std::make_shared<rclcpp::Node>("so101_visual_grasp_gripper_client", helper_options);
  auto client = rclcpp_action::create_client<GripperCommand>(helper, action_name);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(helper);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  bool server_ready = false;
  while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    if (client->wait_for_action_server(std::chrono::milliseconds(250))) {
      server_ready = true;
      break;
    }
    exec.spin_some();
  }

  if (!server_ready) {
    exec.remove_node(helper);
    RCLCPP_ERROR(node->get_logger(), "Gripper action server not available: %s", action_name.c_str());
    return false;
  }

  GripperCommand::Goal goal;
  goal.command.name = {"gripper"};
  goal.command.position = {position};
  goal.command.velocity = {0.0};
  goal.command.effort = {max_effort};

  auto send_fut = client->async_send_goal(goal);
  if (exec.spin_until_future_complete(send_fut, std::chrono::duration<double>(timeout_s)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    exec.remove_node(helper);
    RCLCPP_ERROR(node->get_logger(), "Timed out sending gripper goal.");
    return false;
  }

  auto goal_handle = send_fut.get();
  if (!goal_handle) {
    exec.remove_node(helper);
    RCLCPP_ERROR(node->get_logger(), "Gripper goal rejected.");
    return false;
  }

  auto res_fut = client->async_get_result(goal_handle);
  if (exec.spin_until_future_complete(res_fut, std::chrono::duration<double>(timeout_s)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    exec.remove_node(helper);
    RCLCPP_ERROR(node->get_logger(), "Timed out waiting for gripper result.");
    return false;
  }
  exec.remove_node(helper);

  const auto wrapped = res_fut.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(node->get_logger(), "Gripper action failed (code=%d).", static_cast<int>(wrapped.code));
    return false;
  }
  return true;
}

static bool plan_and_execute_pose(
  moveit::planning_interface::MoveGroupInterface& group,
  const geometry_msgs::msg::PoseStamped& target,
  bool execute,
  bool position_only,
  rclcpp::Logger logger)
{
  group.setStartStateToCurrentState();
  group.clearPoseTargets();
  if (position_only) {
    if (!group.setApproximateJointValueTarget(target)) {
      group.setPositionTarget(target.pose.position.x, target.pose.position.y, target.pose.position.z);
    }
  } else {
    group.setPoseTarget(target);
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto code = group.plan(plan);
  if (code != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Planning failed (code=%d).", code.val);
    return false;
  }
  if (!execute) {
    RCLCPP_INFO(logger, "Plan OK (execute=false).");
    return true;
  }
  const auto exec_code = group.execute(plan);
  if (exec_code != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Execution failed (code=%d).", exec_code.val);
    return false;
  }
  return true;
}

static bool plan_and_execute_named(
  moveit::planning_interface::MoveGroupInterface& group,
  const std::string& name,
  bool execute,
  rclcpp::Logger logger)
{
  group.setStartStateToCurrentState();
  group.clearPoseTargets();
  if (!group.setNamedTarget(name)) {
    RCLCPP_ERROR(logger, "Failed to set named target '%s'.", name.c_str());
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto code = group.plan(plan);
  if (code != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Planning to named target '%s' failed (code=%d).", name.c_str(), code.val);
    return false;
  }
  if (!execute) {
    RCLCPP_INFO(logger, "Named target '%s' plan OK (execute=false).", name.c_str());
    return true;
  }
  const auto exec_code = group.execute(plan);
  if (exec_code != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Execution to named target '%s' failed (code=%d).", name.c_str(), exec_code.val);
    return false;
  }
  return true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "so101_visual_grasp",
    rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));

  auto declare_if_missing = [&](const std::string& name, const auto& def) {
    if (!node->has_parameter(name)) node->declare_parameter(name, def);
  };

  declare_if_missing("move_group_node", std::string{"/move_group"});
  declare_if_missing("arm_group", std::string{"manipulator"});
  declare_if_missing("target_frame", std::string{"base_link"});
  declare_if_missing("pose_topic", std::string{"/vision/aruco/pose_base"});
  declare_if_missing("execute", true);
  declare_if_missing("enable_arm", true);
  declare_if_missing("enable_gripper", true);
  declare_if_missing("wait_pose_timeout_s", 30.0);
  declare_if_missing("pose_sample_window_s", 0.35);
  declare_if_missing("pose_sample_count", 5);
  declare_if_missing("max_pose_spread_m", 0.03);
  declare_if_missing("post_rest_settle_s", 1.0);
  declare_if_missing("pose_transform_timeout_s", 0.1);
  declare_if_missing("pregrasp_offset_m", 0.08);
  declare_if_missing("grasp_x_offset_m", 0.0);
  declare_if_missing("grasp_y_offset_m", 0.0);
  declare_if_missing("grasp_z_offset_m", -0.015);
  declare_if_missing("position_only", true);
  declare_if_missing("go_to_rest_before_grasp", true);
  declare_if_missing("use_marker_orientation", false);
  declare_if_missing("use_rpy", false);
  declare_if_missing("qx", 0.0);
  declare_if_missing("qy", 1.0);
  declare_if_missing("qz", 0.0);
  declare_if_missing("qw", 0.0);
  declare_if_missing("roll", 0.0);
  declare_if_missing("pitch", 0.0);
  declare_if_missing("yaw", 0.0);
  declare_if_missing("planning_time_s", 5.0);
  declare_if_missing("vel_scaling", 0.2);
  declare_if_missing("acc_scaling", 0.2);
  declare_if_missing("gripper_action_name", std::string{"/follower/gripper_controller/gripper_cmd"});
  declare_if_missing("gripper_open_pos", 1.5);
  declare_if_missing("gripper_closed_pos", -0.16);
  declare_if_missing("gripper_max_effort", 0.0);
  declare_if_missing("action_timeout_s", 10.0);

  const auto move_group_node = node->get_parameter("move_group_node").as_string();
  const auto arm_group = node->get_parameter("arm_group").as_string();
  const auto target_frame = node->get_parameter("target_frame").as_string();
  const auto pose_topic = node->get_parameter("pose_topic").as_string();
  const auto execute = node->get_parameter("execute").as_bool();
  const auto enable_arm = node->get_parameter("enable_arm").as_bool();
  const auto enable_gripper = node->get_parameter("enable_gripper").as_bool();
  const auto wait_pose_timeout_s = node->get_parameter("wait_pose_timeout_s").as_double();
  const auto pose_sample_window_s = node->get_parameter("pose_sample_window_s").as_double();
  const auto pose_sample_count = std::max<int64_t>(1, node->get_parameter("pose_sample_count").as_int());
  const auto max_pose_spread_m = node->get_parameter("max_pose_spread_m").as_double();
  const auto post_rest_settle_s = node->get_parameter("post_rest_settle_s").as_double();
  const auto pose_transform_timeout_s = node->get_parameter("pose_transform_timeout_s").as_double();
  const auto pregrasp_offset_m = node->get_parameter("pregrasp_offset_m").as_double();
  const auto grasp_x_offset_m = node->get_parameter("grasp_x_offset_m").as_double();
  const auto grasp_y_offset_m = node->get_parameter("grasp_y_offset_m").as_double();
  const auto grasp_z_offset_m = node->get_parameter("grasp_z_offset_m").as_double();
  const auto position_only = node->get_parameter("position_only").as_bool();
  const auto go_to_rest_before_grasp = node->get_parameter("go_to_rest_before_grasp").as_bool();
  const auto use_marker_orientation = node->get_parameter("use_marker_orientation").as_bool();
  const auto use_rpy = node->get_parameter("use_rpy").as_bool();
  const auto qx = node->get_parameter("qx").as_double();
  const auto qy = node->get_parameter("qy").as_double();
  const auto qz = node->get_parameter("qz").as_double();
  const auto qw = node->get_parameter("qw").as_double();
  const auto roll = node->get_parameter("roll").as_double();
  const auto pitch = node->get_parameter("pitch").as_double();
  const auto yaw = node->get_parameter("yaw").as_double();
  const auto planning_time_s = node->get_parameter("planning_time_s").as_double();
  const auto vel_scaling = node->get_parameter("vel_scaling").as_double();
  const auto acc_scaling = node->get_parameter("acc_scaling").as_double();
  const auto gripper_action_name = node->get_parameter("gripper_action_name").as_string();
  const auto gripper_open_pos = node->get_parameter("gripper_open_pos").as_double();
  const auto gripper_closed_pos = node->get_parameter("gripper_closed_pos").as_double();
  const auto gripper_max_effort = node->get_parameter("gripper_max_effort").as_double();
  const auto action_timeout_s = node->get_parameter("action_timeout_s").as_double();

  if (!ensure_robot_model_params_from_move_group(node, move_group_node)) {
    rclcpp::shutdown();
    return 2;
  }

  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer, node, false);

  struct PoseState
  {
    std::mutex mutex;
    std::condition_variable cv;
    std::optional<geometry_msgs::msg::PoseStamped> latest;
    std::deque<geometry_msgs::msg::PoseStamped> samples;
    std::optional<rclcpp::Time> min_stamp;
    std::size_t update_count{0};
  } pose_state;

  auto pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic,
    10,
    [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(pose_state.mutex);
      if (pose_state.min_stamp.has_value()) {
        const rclcpp::Time stamp(msg->header.stamp);
        if (stamp < pose_state.min_stamp.value()) {
          return;
        }
      }
      pose_state.latest = *msg;
      pose_state.samples.push_back(*msg);
      const std::size_t max_samples = static_cast<std::size_t>(std::max<int64_t>(pose_sample_count, 1) * 4);
      while (pose_state.samples.size() > max_samples) {
        pose_state.samples.pop_front();
      }
      pose_state.update_count++;
      pose_state.cv.notify_all();
    });

  (void)pose_sub;

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  std::thread spin_thread([&exec]() { exec.spin(); });

  auto fail = [&](const std::string& what) {
    RCLCPP_ERROR(node->get_logger(), "%s", what.c_str());
    exec.cancel();
    rclcpp::shutdown();
    if (spin_thread.joinable()) spin_thread.join();
    return 1;
  };

  moveit::planning_interface::MoveGroupInterface arm(node, arm_group);
  arm.setPoseReferenceFrame(target_frame);
  arm.setPlanningTime(planning_time_s);
  arm.setMaxVelocityScalingFactor(vel_scaling);
  arm.setMaxAccelerationScalingFactor(acc_scaling);

  auto wait_for_stable_pose = [&](bool require_fresh) -> std::optional<geometry_msgs::msg::PoseStamped> {
    std::vector<geometry_msgs::msg::PoseStamped> pose_samples;
    std::optional<geometry_msgs::msg::PoseStamped> pose_opt;
    {
      std::unique_lock<std::mutex> lock(pose_state.mutex);
      if (require_fresh) {
        pose_state.latest.reset();
        pose_state.samples.clear();
        pose_state.min_stamp = node->now();
      }
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(wait_pose_timeout_s);
      while (!pose_state.latest.has_value()) {
        if (pose_state.cv.wait_until(lock, deadline) == std::cv_status::timeout) {
          break;
        }
      }
      if (pose_state.latest.has_value()) {
        pose_opt = pose_state.latest;
        const auto sample_deadline =
          std::chrono::steady_clock::now() + std::chrono::duration<double>(pose_sample_window_s);
        const auto start_update_count = pose_state.update_count;
        while (std::chrono::steady_clock::now() < sample_deadline &&
               static_cast<int64_t>(pose_state.samples.size()) < pose_sample_count)
        {
          if (pose_state.cv.wait_until(lock, sample_deadline) == std::cv_status::timeout) {
            break;
          }
          if (pose_state.update_count == start_update_count) {
            continue;
          }
        }

        const auto available = static_cast<int64_t>(pose_state.samples.size());
        const auto keep = std::max<int64_t>(1, std::min<int64_t>(pose_sample_count, available));
        pose_samples.reserve(static_cast<std::size_t>(keep));
        for (auto it = pose_state.samples.end() - keep; it != pose_state.samples.end(); ++it) {
          pose_samples.push_back(*it);
        }
      }
    }

    if (!pose_opt.has_value()) {
      return std::nullopt;
    }

    geometry_msgs::msg::PoseStamped grasp = pose_opt.value();
    if (!pose_samples.empty()) {
      std::vector<double> xs;
      std::vector<double> ys;
      std::vector<double> zs;
      xs.reserve(pose_samples.size());
      ys.reserve(pose_samples.size());
      zs.reserve(pose_samples.size());
      for (const auto& sample : pose_samples) {
        xs.push_back(sample.pose.position.x);
        ys.push_back(sample.pose.position.y);
        zs.push_back(sample.pose.position.z);
      }

      const double median_x = median_of(xs);
      const double median_y = median_of(ys);
      const double median_z = median_of(zs);
      double max_spread = 0.0;
      for (const auto& sample : pose_samples) {
        const double dx = sample.pose.position.x - median_x;
        const double dy = sample.pose.position.y - median_y;
        const double dz = sample.pose.position.z - median_z;
        max_spread = std::max(max_spread, std::sqrt(dx * dx + dy * dy + dz * dz));
      }
      if (max_pose_spread_m > 0.0 && max_spread > max_pose_spread_m) {
        RCLCPP_ERROR(
          node->get_logger(),
          "Pose samples are unstable on '%s' (max spread=%.3fm).",
          pose_topic.c_str(),
          max_spread);
        return std::nullopt;
      }
      grasp = pose_samples.back();
      grasp.pose.position.x = median_x;
      grasp.pose.position.y = median_y;
      grasp.pose.position.z = median_z;
      RCLCPP_INFO(
        node->get_logger(),
        "Using %zu pose samples (window=%.2fs, spread=%.3fm).",
        pose_samples.size(),
        pose_sample_window_s,
        max_spread);
    }

    if (grasp.header.frame_id != target_frame) {
      try {
        grasp = tf_buffer.transform(
          grasp,
          target_frame,
          tf2::durationFromSec(pose_transform_timeout_s));
      } catch (const std::exception& exc) {
        RCLCPP_ERROR(
          node->get_logger(),
          "Failed to transform target pose to %s: %s",
          target_frame.c_str(),
          exc.what());
        return std::nullopt;
      }
    }

    if (!use_marker_orientation) {
      if (use_rpy) {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q.normalize();
        grasp.pose.orientation.x = q.x();
        grasp.pose.orientation.y = q.y();
        grasp.pose.orientation.z = q.z();
        grasp.pose.orientation.w = q.w();
      } else {
        const bool all_zero = (std::fabs(qx) < 1e-12) && (std::fabs(qy) < 1e-12) &&
                              (std::fabs(qz) < 1e-12) && (std::fabs(qw) < 1e-12);
        grasp.pose.orientation.x = all_zero ? 0.0 : qx;
        grasp.pose.orientation.y = all_zero ? 0.0 : qy;
        grasp.pose.orientation.z = all_zero ? 0.0 : qz;
        grasp.pose.orientation.w = all_zero ? 1.0 : qw;
      }
    }

    grasp.pose.position.x += grasp_x_offset_m;
    grasp.pose.position.y += grasp_y_offset_m;
    grasp.pose.position.z += grasp_z_offset_m;
    return grasp;
  };

  if (enable_gripper && execute) {
    RCLCPP_INFO(node->get_logger(), "Opening gripper...");
    if (!send_gripper_command(node, gripper_action_name, gripper_open_pos, gripper_max_effort, action_timeout_s)) {
      return fail("Failed to open gripper.");
    }
  } else if (enable_gripper && !execute) {
    RCLCPP_INFO(node->get_logger(), "Dry-run mode: skip gripper open action.");
  }

  if (enable_arm) {
    if (go_to_rest_before_grasp) {
      RCLCPP_INFO(node->get_logger(), "Moving to rest pose first...");
      if (!plan_and_execute_named(arm, "rest", execute, node->get_logger())) {
        return fail("Failed to move to rest pose.");
      }
      if (execute && post_rest_settle_s > 0.0) {
        RCLCPP_INFO(node->get_logger(), "Waiting %.2fs for vision to settle after rest...", post_rest_settle_s);
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(post_rest_settle_s)));
      }
    }
  }

  auto grasp_opt = wait_for_stable_pose(go_to_rest_before_grasp && enable_arm);
  if (!grasp_opt.has_value()) {
    return fail(std::string("Timed out waiting for a fresh stable pose on '") + pose_topic + "'.");
  }

  geometry_msgs::msg::PoseStamped grasp = grasp_opt.value();
  geometry_msgs::msg::PoseStamped pregrasp = grasp;
  pregrasp.pose.position.z += pregrasp_offset_m;

  RCLCPP_INFO(
    node->get_logger(),
    "Target pose: frame='%s' x=%.3f y=%.3f z=%.3f",
    grasp.header.frame_id.c_str(),
    grasp.pose.position.x,
    grasp.pose.position.y,
    grasp.pose.position.z);
  RCLCPP_INFO(
    node->get_logger(),
    "Pregrasp pose: x=%.3f y=%.3f z=%.3f",
    pregrasp.pose.position.x,
    pregrasp.pose.position.y,
    pregrasp.pose.position.z);
  RCLCPP_INFO(
    node->get_logger(),
    "Orientation mode: marker=%s rpy=%s quat=[%.3f, %.3f, %.3f, %.3f] rpy=[%.3f, %.3f, %.3f]",
    use_marker_orientation ? "true" : "false",
    use_rpy ? "true" : "false",
    grasp.pose.orientation.x,
    grasp.pose.orientation.y,
    grasp.pose.orientation.z,
    grasp.pose.orientation.w,
    roll,
    pitch,
    yaw);

  if (enable_arm) {
    RCLCPP_INFO(node->get_logger(), "Moving to pregrasp...");
    if (!plan_and_execute_pose(arm, pregrasp, execute, position_only, node->get_logger())) {
      return fail("Failed to move to pregrasp.");
    }
    RCLCPP_INFO(node->get_logger(), "Moving to grasp...");
    if (!plan_and_execute_pose(arm, grasp, execute, position_only, node->get_logger())) {
      return fail("Failed to move to grasp.");
    }
  }

  if (enable_gripper && execute) {
    RCLCPP_INFO(node->get_logger(), "Closing gripper...");
    if (!send_gripper_command(node, gripper_action_name, gripper_closed_pos, gripper_max_effort, action_timeout_s)) {
      return fail("Failed to close gripper.");
    }
  }

  if (enable_arm) {
    RCLCPP_INFO(node->get_logger(), "Retreating...");
    if (!plan_and_execute_pose(arm, pregrasp, execute, position_only, node->get_logger())) {
      return fail("Failed to retreat.");
    }
  }

  RCLCPP_INFO(node->get_logger(), "Visual grasp sequence completed.");

  exec.cancel();
  rclcpp::shutdown();
  if (spin_thread.joinable()) spin_thread.join();
  return 0;
}
