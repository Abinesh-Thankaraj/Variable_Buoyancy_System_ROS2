#include <auv_control/controller_io.h>
#include <chrono>
#include <algorithm>  // For std::clamp

using namespace auv_control;
using namespace std::chrono_literals;

// Utility functions remain unchanged
void twist2Eigen(const geometry_msgs::msg::Twist &twist, Vector6d &vel) {
  vel[0] = twist.linear.x;
  vel[1] = twist.linear.y;
  vel[2] = twist.linear.z;
  vel[3] = twist.angular.x;
  vel[4] = twist.angular.y;
  vel[5] = twist.angular.z;
}

void wrench2Eigen(const geometry_msgs::msg::Wrench &wrench, Vector6d &vec) {
  vec[0] = wrench.force.x;
  vec[1] = wrench.force.y;
  vec[2] = wrench.force.z;
  vec[3] = wrench.torque.x;
  vec[4] = wrench.torque.y;
  vec[5] = wrench.torque.z;
}

Vector6d ControllerIO::Pose::toSE3() const {
  Vector6d se3_error;
  se3_error.head<3>() = t;
  const Eigen::AngleAxisd tu(q);
  se3_error.tail<3>() = tu.angle() * tu.axis();
  return se3_error;
}

ControllerIO::ControllerIO(std::string name, rclcpp::NodeOptions options)
    : Node(name, options),
    tf_buffer(get_clock()), tf_listener(tf_buffer) 
{
  // Initialize control frame
  control_frame = get_namespace();
  if(const auto slash{control_frame.find_last_of('/')}; slash == control_frame.npos) {
    control_frame = "base_link";
  } else {
    control_frame = control_frame.substr(slash+1) + "/base_link";
  }
  control_frame = declare_parameter<std::string>("control_frame", control_frame);

  // Initialize hydrodynamics if enabled
  if(declare_parameter("use_hydro", true)) {
    hydro = Hydrodynamics(control_frame, this);
  }

  // Alignment threshold for line-of-sight control
  align_thr = declare_parameter("align_thr", -1.);

  // Parse robot description and initialize thrusters
  const auto links{allocator.parseRobotDescription(this, control_frame)};
  std::transform(links.begin(), links.end(), std::back_inserter(cmd.name), 
                [](const auto &link){return link.joint;});
  dofs = cmd.name.size();
  cmd.effort.resize(dofs, 0);

  // Setup publishers
  if(declare_parameter("publish_joint_state", true)) {
    cmd_js_pub = create_publisher<JointState>("cmd_thrust", 5); 
  }

  if(get_parameter("use_sim_time").as_bool()) {
    for(const auto &name: cmd.name) {
      const auto topic{"cmd_" + name};
      cmd_gz_pub.push_back(create_publisher<Float64>(topic, 5));
    }
  }

  // Setup subscribers
  current_estim = Vector3d::Zero();
  current_sub = create_subscription<Vector3>("current_estim", 10, 
      [&](Vector3::SharedPtr msg) {
        current_estim[0] = msg->x;
        current_estim[1] = msg->y;
        current_estim[2] = msg->z;
      });

  pose_sub = create_subscription<PoseStamped>("cmd_pose", 10,
      [&](PoseStamped::SharedPtr msg) {
        pose_setpoint.from(msg->pose.position, msg->pose.orientation);
        pose_setpoint.frame = msg->header.frame_id;
        pose_setpoint.time = get_clock()->now().seconds();
      });

  vel_sub = create_subscription<TwistStamped>("cmd_vel", 10,
      [&](TwistStamped::SharedPtr msg) {
        twist2Eigen(msg->twist, vel_setpoint);
        vel_setpoint.frame = msg->header.frame_id;
        vel_setpoint.time = get_clock()->now().seconds();
      });

  wrench_sub = create_subscription<Wrench>("cmd_wrench", 1,
      [&](Wrench::SharedPtr msg) {
        wrench2Eigen(*msg, wrench_setpoint);
        wrench_setpoint.time = get_clock()->now().seconds();
      });

  odom_sub = create_subscription<Odometry>("odom", 10,
      [&](Odometry::SharedPtr msg) {
        if(!odom_twist.has_value()) {
          odom_twist.emplace();
        }
        twist2Eigen(msg->twist.twist, *odom_twist);
      });

  // Setup service
  control_srv = create_service<ControlMode>("control_mode",
      [&](const ControlMode::Request::SharedPtr request,
          [[maybe_unused]] ControlMode::Response::SharedPtr response) {
        control_mode = request->mode;
      });

  // Initialize control timer
  const auto cmd_period = std::chrono::milliseconds(
      declareParameterBounded("control_period_ms", 100, 1, 1000, 1));
  dt = cmd_period.count() / 1000.;
  vel_filter.init(6, .5/dt, dt);
  cmd_timer = create_wall_timer(cmd_period, [&]() { publish(computeThrusts()); });
}

Vector6d ControllerIO::twist(const Eigen::Quaterniond& q) {
  if(odom_twist.has_value()) {
    vel_filter.filter(odom_twist.value());
    return odom_twist.value();
  }

  Vector6d twist{Vector6d::Zero()};
#ifdef AUV_CONTROL_WITH_LOOKUP_VELOCITY
  try {
    const auto stamped{tf_buffer.lookupVelocity("world", control_frame,
                     tf2::timeFromSec(get_clock()->now().seconds()-dt), 100ms)};
    twist2Eigen(stamped.velocity, twist);
    twist.head<3>() = q * twist.head<3>();
    twist.tail<3>() = q * twist.tail<3>();
  } catch(const tf2::TransformException &ex) {
    RCLCPP_ERROR(get_logger(), "Cannot get velocity: %s", ex.what());
  }
  vel_filter.filter(twist);
#endif
  return twist;
}

Eigen::VectorXd ControllerIO::computeThrusts() {
  const auto now_s{get_clock()->now().seconds()};
  const auto pose_timeout{now_s - pose_setpoint.time > pose_setpoint_timeout};
  const auto vel_timeout{now_s - vel_setpoint.time > vel_setpoint_timeout};
  const auto wrench_timeout{now_s - wrench_setpoint.time > pose_setpoint_timeout};

  // Return zero thrust if no active commands
  if(pose_timeout && vel_timeout && wrench_timeout) {
    return Eigen::VectorXd::Zero(dofs);
  }

  Vector6d wrench;

  // Handle manual wrench control
  if(!wrench_timeout) {
    wrench = wrench_setpoint;
    
    // Apply hydro compensation if enabled
    if(hydro) {
      const auto q{relPose("world").q};
      hydro->compensate(wrench, q, twist(q), current_estim);
    }
  } 
  // Handle automatic control
  else {
    // Compute position error
    Vector6d se3_error;
    if(pose_timeout || control_mode == ControlMode::Request::VELOCITY) {
      se3_error.setZero();
    } else if(pose_setpoint.frame == control_frame) {
      se3_error = pose_setpoint.toSE3();
    } else {
      se3_error = (relPose(pose_setpoint.frame) * pose_setpoint).toSE3();
    }

    // Compute velocity setpoint
    Vector6d twist_setpoint;
    if(vel_timeout) {
      twist_setpoint.setZero();
    } else if(vel_setpoint.frame == control_frame || vel_setpoint.isApproxToConstant(0, 1e-3)) {
      twist_setpoint = vel_setpoint;
    } else {
      relPose(vel_setpoint.frame).rotate2(vel_setpoint, twist_setpoint);
    }

    // Handle special control modes
    if(control_mode == ControlMode::Request::DEPTH) {
      se3_error[0] = se3_error[1] = se3_error[5] = 0.;
    }

    // Line-of-sight guidance
    if(align_thr > 0.) {
      static auto los_phase{true};
      const auto err{se3_error.head<2>().norm()};
      if(err > 2*align_thr) {
        los_phase = true;
      } else if(err < align_thr) {
        los_phase = false;
      }

      if(los_phase) {
        se3_error[5] = .5*atan2(se3_error[1], se3_error[0]);
      }
    }

    // Compute control wrench
    const auto q{relPose("world").q};
    const auto current_twist{this->twist(q)};
    wrench = computeWrench(se3_error, current_twist, twist_setpoint);
    
    // Apply hydro compensation if enabled
    if(hydro) {
      hydro->compensate(wrench, q, current_twist, current_estim);
    }
  }

  // ========== CRITICAL: CLAMP CONTROL OUTPUTS ==========
  // Z force (heave): ±20N (2 ballast × 10N each)
  wrench[2] = std::clamp(wrench[2], -20.0, 0.0);
  
  // Pitch torque: ±14Nm (0.7m moment arm × 10N × 2 thrusters)
  wrench[4] = std::clamp(wrench[4], -14.0, 0.0);

  // Solve for individual thruster forces
  return allocator.solveWrench(wrench);
}

void ControllerIO::publish(const Eigen::VectorXd &thrusts) {
  // Publish joint state message
  std::copy(thrusts.data(), thrusts.data() + dofs, cmd.effort.begin());
  if(cmd_js_pub) {
    cmd.header.stamp = get_clock()->now();
    cmd_js_pub->publish(cmd);
  }

  // Publish Gazebo commands
  if(!cmd_gz_pub.empty()) {
    static Float64 cmd_gz;
    for(size_t i = 0; i < dofs; ++i) {
      cmd_gz.data = thrusts[i];
      cmd_gz_pub[i]->publish(cmd_gz);
    }
  }
}

ControllerIO::Pose ControllerIO::relPose(const std::string &frame) const {
  Pose rel_pose;
  if(tf_buffer.canTransform(control_frame, frame, tf2::TimePointZero, 10ms)) {
    const auto tf{tf_buffer.lookupTransform(control_frame, frame, 
                  tf2::TimePointZero, 10ms).transform};
    rel_pose.from(tf.translation, tf.rotation);
  }
  return rel_pose;
}
