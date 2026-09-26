#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>

namespace gazebo {

// Front-wheel-steering model with fixed, table-based keyboard steering.
//
// The plugin drives the four wheel joints with the theoretical no-slip
// Ackermann speeds and locks the front steering joints to stable table values
// for the keyboard's 0/20/40 degree commands.
// The /sim/ground_truth/odom topic is evaluation-only; publish_tf is
// deliberately false in the generated URDF so this plugin cannot compete with
// the runtime odometry publisher.
class InspectionAckermann : public ModelPlugin {
  physics::ModelPtr model_;
  physics::JointPtr wheels_[4];  // FL, FR, RL, RR
  physics::JointPtr steer_[2];   // FL, FR
  event::ConnectionPtr update_;
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::CallbackQueue queue_;
  ros::Subscriber sub_;
  ros::Publisher truth_odom_, joints_;
  std::unique_ptr<tf::TransformBroadcaster> tf_;

  double L_, front_track_, rear_track_, radius_;
  double max_wheel_angle_, wheel_torque_, steer_torque_;
  double vmax_, accel_, brake_, steer_rate_, steer_kp_;
  int publish_tf_;

  double v_cmd_ = 0.0;
  double curvature_cmd_ = 0.0;
  double applied_v_ = 0.0;
  double applied_delta_ = 0.0;  // centre/front-axle reference angle
  double last_time_ = 0.0;
  double last_pub_ = 0.0;
  ros::WallTime last_command_;
  bool warned_large_dt_ = false;

  static double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
  }

  double read(sdf::ElementPtr sdf, const std::string& key, double fallback) {
    return sdf->HasElement(key) ? sdf->Get<double>(key) : fallback;
  }

  void command(const geometry_msgs::Twist::ConstPtr& msg) {
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z)) {
      v_cmd_ = 0.0;
      return;
    }
    v_cmd_ = clamp(msg->linear.x, -vmax_, vmax_);
    // Decode paired Twist values BEFORE acceleration/speed saturation. Slowing
    // down must not change the requested turn radius. A zero-speed stop holds
    // steering while braking, then returns to centre after the wheels stop.
    if (std::abs(msg->linear.x) > 1e-4) {
      const double r_min = L_ / std::tan(max_wheel_angle_) + front_track_ / 2.0;
      curvature_cmd_ = clamp(msg->angular.z / msg->linear.x, -1.0 / r_min, 1.0 / r_min);
    }
    last_command_ = ros::WallTime::now();
  }

  void set_wheel_velocity(const physics::JointPtr& joint, double linear_speed) {
    joint->SetParam("fmax", 0, wheel_torque_);
    joint->SetParam("vel", 0, linear_speed / radius_);
  }

  void set_steer_target(const physics::JointPtr& joint, double target) {
    target = clamp(target, -max_wheel_angle_, max_wheel_angle_);
    // These are discrete steering targets. Set the joint position directly
    // so tire contact cannot make one wheel oscillate around its target.
    joint->SetPosition(0, target);
    joint->SetParam("fmax", 0, steer_torque_);
    joint->SetParam("vel", 0, 0.0);
  }

 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    if (!ros::isInitialized()) {
      gzerr << "gazebo_ros must initialize ROS first\n";
      return;
    }

    model_ = model;
    L_ = read(sdf, "wheelbase", 0.62);
    front_track_ = read(sdf, "front_track", 0.45);
    rear_track_ = read(sdf, "rear_track", 0.45);
    radius_ = read(sdf, "wheel_radius", 0.09);
    max_wheel_angle_ = read(sdf, "max_wheel_angle", 0.6981317008);
    wheel_torque_ = read(sdf, "wheel_torque", 8.0);
    steer_torque_ = read(sdf, "steer_torque", 60.0);
    vmax_ = read(sdf, "max_speed", 0.35);
    accel_ = read(sdf, "acceleration", 0.30);
    brake_ = read(sdf, "braking", 0.50);
    steer_rate_ = read(sdf, "steer_rate", 2.5);
    steer_kp_ = read(sdf, "steer_kp", 4.0);
    publish_tf_ = sdf->HasElement("publish_tf")
                    ? static_cast<int>(sdf->Get<double>("publish_tf")) : 0;

    const char* wheel_names[] = {
      "front_left_wheel_joint", "front_right_wheel_joint",
      "rear_left_wheel_joint", "rear_right_wheel_joint"};
    for (int i = 0; i < 4; ++i) {
      wheels_[i] = model_->GetJoint(wheel_names[i]);
      if (!wheels_[i]) {
        gzerr << "Required wheel joint missing: " << wheel_names[i] << "\n";
        return;
      }
    }

    steer_[0] = model_->GetJoint("front_left_steer_joint");
    steer_[1] = model_->GetJoint("front_right_steer_joint");
    if (!steer_[0] || !steer_[1]) {
      gzerr << "Required steering joint missing\n";
      return;
    }

    // Set mechanical stops once. Never move the stops to the command angle.
    for (int i = 0; i < 2; ++i) {
      steer_[i]->SetLowStop(0, -max_wheel_angle_);
      steer_[i]->SetHighStop(0, max_wheel_angle_);
      steer_[i]->SetParam("fmax", 0, steer_torque_);
    }

    nh_.reset(new ros::NodeHandle("/sim"));
    // Runtime contract read by navigation: no duplicated launch defaults.
    nh_->setParam("actuator_model/version", 1);
    nh_->setParam("actuator_model/wheelbase", L_);
    nh_->setParam("actuator_model/front_track", front_track_);
    nh_->setParam("actuator_model/wheel_radius", radius_);
    nh_->setParam("actuator_model/acceleration", accel_);
    nh_->setParam("actuator_model/braking", brake_);
    nh_->setParam("actuator_model/steer_rate", steer_rate_);
    nh_->setCallbackQueue(&queue_);
    sub_ = nh_->subscribe("cmd_vel", 1, &InspectionAckermann::command, this);
    truth_odom_ = nh_->advertise<nav_msgs::Odometry>("ground_truth/odom", 5);
    joints_ = nh_->advertise<sensor_msgs::JointState>("joint_states", 5);
    tf_.reset(new tf::TransformBroadcaster());

    last_command_ = ros::WallTime::now();
    last_time_ = model_->GetWorld()->GetSimTime().Double();
    update_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&InspectionAckermann::step, this));

    ROS_INFO("Front steering simulation enabled: shared front angle and four "
             "theoretical wheel speeds; /sim/ground_truth/odom is evaluation-only.");
  }

  void step() {
    queue_.callAvailable(ros::WallDuration(0));

    const double now = model_->GetWorld()->GetSimTime().Double();
    double dt = now - last_time_;
    last_time_ = now;
    if (dt <= 0.0) {
      return;
    }
    if (dt > 0.10) {
      if (!warned_large_dt_) {
        ROS_WARN("Ackermann simulation skipped a large physics interval: %.3f s", dt);
        warned_large_dt_ = true;
      }
      // Do not create a fake pose jump after a paused or overloaded simulator.
      dt = 0.0;
    } else {
      warned_large_dt_ = false;
    }

    if ((ros::WallTime::now() - last_command_).toSec() > 0.60) {
      v_cmd_ = 0.0;
    }

    if (dt > 0.0) {
      const double dv_limit = (std::abs(v_cmd_) > std::abs(applied_v_))
                                ? accel_ * dt : brake_ * dt;
      applied_v_ += clamp(v_cmd_ - applied_v_, -dv_limit, dv_limit);

      const double delta_cmd = (std::abs(v_cmd_) <= 1e-4 && std::abs(applied_v_) <= 1e-4)
                                   ? 0.0 : std::atan(L_ * curvature_cmd_);
      applied_delta_ += clamp(delta_cmd - applied_delta_,
                              -steer_rate_ * dt, steer_rate_ * dt);
    }

    // Use the achieved centre angle, not the command, for wheel geometry.
    const double kappa = std::tan(applied_delta_) / L_;
    double left_delta = applied_delta_;
    double right_delta = applied_delta_;
    double v_fl = applied_v_;
    double v_fr = applied_v_;
    double v_rl = applied_v_;
    double v_rr = applied_v_;

    if (std::abs(kappa) > 1e-8) {
      const double yaw_rate = applied_v_ * kappa;
      const double R = 1.0 / kappa;
      const double y_left = front_track_ / 2.0;
      const double y_right = -front_track_ / 2.0;

      // Fixed Ackermann table for the keyboard's exact 20/40 degree steps.
      // Navigation may send other angles; keep continuous Ackermann geometry
      // for those commands while still using direct joint position control.
      const double sign = (applied_delta_ >= 0.0) ? 1.0 : -1.0;
      const double abs_delta = std::abs(applied_delta_);
      const double d20 = 0.349066;
      const double d40 = 0.698132;
      if (std::abs(abs_delta - d20) < 0.03) {
        const double inner = 0.39674;  // 22.75 degrees
        const double outer = 0.31107;  // 17.82 degrees
        left_delta = (sign > 0.0) ? inner : -outer;
        right_delta = (sign > 0.0) ? outer : -inner;
      } else if (std::abs(abs_delta - d40) < 0.03) {
        const double inner = 0.87850;  // 50.35 degrees, no 40 degree cap
        const double outer = 0.57151;  // 32.75 degrees
        left_delta = (sign > 0.0) ? inner : -outer;
        right_delta = (sign > 0.0) ? outer : -inner;
      } else {
        // atan (rather than atan2) keeps both steering angles in the finite
        // front-wheel range for forward and reverse travel.
        left_delta = std::atan(L_ / (R - y_left));
        right_delta = std::atan(L_ / (R - y_right));
      }

      v_rl = yaw_rate * (R - rear_track_ / 2.0);
      v_rr = yaw_rate * (R + rear_track_ / 2.0);
      v_fl = yaw_rate * (R - y_left) / std::cos(left_delta);
      v_fr = yaw_rate * (R - y_right) / std::cos(right_delta);
    }

    set_steer_target(steer_[0], left_delta);
    set_steer_target(steer_[1], right_delta);
    set_wheel_velocity(wheels_[0], v_fl);
    set_wheel_velocity(wheels_[1], v_fr);
    set_wheel_velocity(wheels_[2], v_rl);
    set_wheel_velocity(wheels_[3], v_rr);

    if (now - last_pub_ < 0.02) {
      return;
    }
    last_pub_ = now;

    const auto pose = model_->GetWorldPose();
    const auto world_linear = model_->GetWorldLinearVel();
    const auto world_angular = model_->GetWorldAngularVel();
    ros::Time stamp; stamp.fromSec(now);

    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "sim_world";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = pose.pos.x;
    odom.pose.pose.position.y = pose.pos.y;
    odom.pose.pose.orientation.x = pose.rot.x;
    odom.pose.pose.orientation.y = pose.rot.y;
    odom.pose.pose.orientation.z = pose.rot.z;
    odom.pose.pose.orientation.w = pose.rot.w;
    const auto local_linear = pose.rot.RotateVectorReverse(world_linear);
    odom.twist.twist.linear.x = local_linear.x;
    odom.twist.twist.linear.y = local_linear.y;
    odom.twist.twist.angular.z = world_angular.z;
    truth_odom_.publish(odom);

    if (publish_tf_) {
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(pose.pos.x, pose.pos.y, 0.0));
      transform.setRotation(tf::Quaternion(pose.rot.x, pose.rot.y,
                                           pose.rot.z, pose.rot.w));
      tf_->sendTransform(tf::StampedTransform(
          transform, stamp, "sim_world", "base_footprint"));
    }

    sensor_msgs::JointState state;
    state.header.stamp = stamp;
    for (int i = 0; i < 4; ++i) {
      state.name.push_back(wheels_[i]->GetName());
      state.position.push_back(wheels_[i]->GetAngle(0).Radian());
      state.velocity.push_back(wheels_[i]->GetVelocity(0));
    }
    for (int i = 0; i < 2; ++i) {
      state.name.push_back(steer_[i]->GetName());
      state.position.push_back(steer_[i]->GetAngle(0).Radian());
      state.velocity.push_back(steer_[i]->GetVelocity(0));
    }
    joints_.publish(state);
  }
};

GZ_REGISTER_MODEL_PLUGIN(InspectionAckermann)
}
