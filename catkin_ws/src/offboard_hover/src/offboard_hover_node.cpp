#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace {

enum class FlightPhase {
  ARMING,
  PRESTREAM_SETPOINTS,
  ENTERING_OFFBOARD,
  TAKING_OFF,
  WAITING_FOR_EGO,
  TRACKING_EGO
};

mavros_msgs::State current_state;
geometry_msgs::PoseStamped latest_lidar_pose;
quadrotor_msgs::PositionCommand latest_ego_command;

tf2::Vector3 lidar_origin_position;
tf2::Quaternion lidar_origin_attitude;
double lidar_origin_yaw = 0.0;

ros::Time last_odom_receive_time;
ros::Time last_ego_receive_time;
bool lidar_origin_initialized = false;
bool lidar_odom_received = false;
bool ego_command_received = false;

const char *phaseName(FlightPhase phase)
{
  switch (phase) {
    case FlightPhase::ARMING:
      return "ARMING";
    case FlightPhase::PRESTREAM_SETPOINTS:
      return "PRESTREAM_SETPOINTS";
    case FlightPhase::ENTERING_OFFBOARD:
      return "ENTERING_OFFBOARD";
    case FlightPhase::TAKING_OFF:
      return "TAKING_OFF";
    case FlightPhase::WAITING_FOR_EGO:
      return "WAITING_FOR_EGO";
    case FlightPhase::TRACKING_EGO:
      return "TRACKING_EGO";
  }
  return "UNKNOWN";
}

double normalizeAngle(double angle)
{
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

geometry_msgs::Quaternion yawQuaternion(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  q.normalize();

  geometry_msgs::Quaternion result;
  result.x = q.x();
  result.y = q.y();
  result.z = q.z();
  result.w = q.w();
  return result;
}

double quaternionYaw(const geometry_msgs::Quaternion &msg)
{
  tf2::Quaternion q(msg.x, msg.y, msg.z, msg.w);
  if (q.length2() < 1e-8) {
    return 0.0;
  }
  q.normalize();

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

void stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
  current_state = *msg;
}

bool isFiniteOdometry(const nav_msgs::Odometry &msg)
{
  const geometry_msgs::Point &p = msg.pose.pose.position;
  const geometry_msgs::Quaternion &q = msg.pose.pose.orientation;

  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
         std::isfinite(q.w);
}

void lidarOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  if (!isFiniteOdometry(*msg)) {
    ROS_WARN_THROTTLE(1.0, "Ignoring non-finite LiDAR odometry");
    return;
  }

  const geometry_msgs::Point &p = msg->pose.pose.position;
  const geometry_msgs::Quaternion &q_msg = msg->pose.pose.orientation;

  tf2::Quaternion current_attitude(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  if (current_attitude.length2() < 1e-8) {
    ROS_WARN_THROTTLE(1.0, "Ignoring LiDAR odometry with invalid quaternion");
    return;
  }
  current_attitude.normalize();

  const tf2::Vector3 current_position(p.x, p.y, p.z);

  if (!lidar_origin_initialized) {
    lidar_origin_position = current_position;
    lidar_origin_attitude = current_attitude;
    lidar_origin_attitude.normalize();

    double origin_roll = 0.0;
    double origin_pitch = 0.0;
    tf2::Matrix3x3(lidar_origin_attitude)
        .getRPY(origin_roll, origin_pitch, lidar_origin_yaw);

    lidar_origin_initialized = true;
    ROS_INFO("LiDAR odometry origin fixed at [%.3f, %.3f, %.3f], yaw %.3f rad",
             p.x, p.y, p.z, lidar_origin_yaw);
  }

  // Express Fast-LIO odometry relative to the takeoff pose. The same
  // transform is applied to EGO commands before forwarding them to PX4.
  const tf2::Quaternion origin_inverse = lidar_origin_attitude.inverse();
  const tf2::Vector3 relative_position =
      tf2::quatRotate(origin_inverse, current_position - lidar_origin_position);

  tf2::Quaternion relative_attitude = origin_inverse * current_attitude;
  relative_attitude.normalize();

  latest_lidar_pose.header.frame_id = "map";
  latest_lidar_pose.header.stamp = msg->header.stamp.isZero()
                                      ? ros::Time::now()
                                      : msg->header.stamp;
  latest_lidar_pose.pose.position.x = relative_position.x();
  latest_lidar_pose.pose.position.y = relative_position.y();
  latest_lidar_pose.pose.position.z = relative_position.z();
  latest_lidar_pose.pose.orientation.x = relative_attitude.x();
  latest_lidar_pose.pose.orientation.y = relative_attitude.y();
  latest_lidar_pose.pose.orientation.z = relative_attitude.z();
  latest_lidar_pose.pose.orientation.w = relative_attitude.w();

  last_odom_receive_time = ros::Time::now();
  lidar_odom_received = true;
}

bool isFiniteEgoCommand(const quadrotor_msgs::PositionCommand &msg)
{
  return std::isfinite(msg.position.x) && std::isfinite(msg.position.y) &&
         std::isfinite(msg.position.z) && std::isfinite(msg.yaw);
}

void egoCommandCallback(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
  if (msg->trajectory_flag !=
      quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY) {
    ROS_WARN_THROTTLE(1.0, "Ignoring EGO command with trajectory flag %u",
                      static_cast<unsigned>(msg->trajectory_flag));
    return;
  }

  if (!isFiniteEgoCommand(*msg)) {
    ROS_WARN_THROTTLE(1.0, "Ignoring non-finite EGO position command");
    return;
  }

  latest_ego_command = *msg;
  last_ego_receive_time = ros::Time::now();
  ego_command_received = true;
}

bool odometryIsFresh(const ros::Time &now, double timeout)
{
  return lidar_odom_received &&
         (now - last_odom_receive_time) <= ros::Duration(timeout);
}

bool egoCommandIsFresh(const ros::Time &now, double timeout)
{
  return ego_command_received &&
         (now - last_ego_receive_time) <= ros::Duration(timeout);
}

geometry_msgs::PoseStamped makeHoverTarget(double height)
{
  geometry_msgs::PoseStamped target;
  target.header.frame_id = "map";
  target.pose.position.x = 0.0;
  target.pose.position.y = 0.0;
  target.pose.position.z = height;
  target.pose.orientation = yawQuaternion(0.0);
  return target;
}

geometry_msgs::PoseStamped makeCurrentHoldTarget()
{
  geometry_msgs::PoseStamped target;
  target.header.frame_id = "map";
  target.pose.position = latest_lidar_pose.pose.position;
  target.pose.orientation =
      yawQuaternion(quaternionYaw(latest_lidar_pose.pose.orientation));
  return target;
}

geometry_msgs::PoseStamped egoCommandToTarget(
    const quadrotor_msgs::PositionCommand &command,
    bool ego_command_is_relative)
{
  geometry_msgs::PoseStamped target;
  target.header.frame_id = "map";

  if (ego_command_is_relative) {
    target.pose.position = command.position;
    target.pose.orientation = yawQuaternion(command.yaw);
    return target;
  }

  // EGO normally plans in the same world frame as its odometry input. When
  // that input is raw Fast-LIO /Odometry, convert EGO's world command to the
  // takeoff-relative frame sent to PX4.
  const tf2::Vector3 ego_world_position(
      command.position.x, command.position.y, command.position.z);
  const tf2::Vector3 relative_position = tf2::quatRotate(
      lidar_origin_attitude.inverse(),
      ego_world_position - lidar_origin_position);

  target.pose.position.x = relative_position.x();
  target.pose.position.y = relative_position.y();
  target.pose.position.z = relative_position.z();
  target.pose.orientation =
      yawQuaternion(normalizeAngle(command.yaw - lidar_origin_yaw));
  return target;
}

double positionError(const geometry_msgs::PoseStamped &a,
                     const geometry_msgs::PoseStamped &b)
{
  const double dx = a.pose.position.x - b.pose.position.x;
  const double dy = a.pose.position.y - b.pose.position.y;
  const double dz = a.pose.position.z - b.pose.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Keep the OFFBOARD proof-of-life stream independent from the state machine
// and synchronous MAVROS service calls. Wall time is intentional: when Gazebo
// runs below real time, the companion-computer link must still publish at a
// steady rate and must never be interrupted by an ARM or mode request.
class SetpointStreamer {
 public:
  SetpointStreamer(const ros::Publisher &publisher, double rate_hz)
      : publisher_(publisher),
        enabled_(false),
        running_(true),
        rate_hz_(rate_hz),
        worker_(&SetpointStreamer::run, this)
  {
  }

  ~SetpointStreamer()
  {
    running_.store(false);
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  SetpointStreamer(const SetpointStreamer &) = delete;
  SetpointStreamer &operator=(const SetpointStreamer &) = delete;

  void setTarget(const geometry_msgs::PoseStamped &target)
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    target_ = target;
  }

  void setEnabled(bool enabled)
  {
    enabled_.store(enabled);
  }

 private:
  void run()
  {
    ros::WallRate rate(rate_hz_);
    while (running_.load() && ros::ok()) {
      if (enabled_.load()) {
        geometry_msgs::PoseStamped message;
        {
          std::lock_guard<std::mutex> lock(target_mutex_);
          message = target_;
        }
        message.header.stamp = ros::Time::now();
        publisher_.publish(message);
      }
      rate.sleep();
    }
  }

  ros::Publisher publisher_;
  std::mutex target_mutex_;
  geometry_msgs::PoseStamped target_;
  std::atomic<bool> enabled_;
  std::atomic<bool> running_;
  double rate_hz_;
  std::thread worker_;
};

}  // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "offboard_hover_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string odom_topic;
  std::string ego_cmd_topic;
  double hover_height = 1.5;
  double odom_timeout = 0.5;
  double ego_timeout = 0.5;
  double prestream_time = 2.0;
  double request_interval = 1.0;
  double hover_tolerance = 0.15;
  double hover_stable_time = 2.0;
  bool ego_command_is_relative = false;
  bool publish_vision_pose = false;

  private_nh.param<std::string>("odom_topic", odom_topic, "/Odometry");
  private_nh.param<std::string>(
      "ego_cmd_topic", ego_cmd_topic, "/planning/pos_cmd");
  private_nh.param("hover_height", hover_height, 1.5);
  private_nh.param("odom_timeout", odom_timeout, 0.5);
  private_nh.param("ego_timeout", ego_timeout, 0.5);
  private_nh.param("prestream_time", prestream_time, 2.0);
  private_nh.param("request_interval", request_interval, 1.0);
  private_nh.param("hover_tolerance", hover_tolerance, 0.15);
  private_nh.param("hover_stable_time", hover_stable_time, 2.0);
  private_nh.param(
      "ego_command_is_relative", ego_command_is_relative, false);
  private_nh.param("publish_vision_pose", publish_vision_pose, false);

  ros::Subscriber state_sub =
      nh.subscribe<mavros_msgs::State>("/mavros/state", 10, stateCallback);
  ros::Subscriber odom_sub =
      nh.subscribe<nav_msgs::Odometry>(odom_topic, 50, lidarOdomCallback);
  ros::Subscriber ego_cmd_sub =
      nh.subscribe<quadrotor_msgs::PositionCommand>(
          ego_cmd_topic, 100, egoCommandCallback);

  ros::Publisher vision_pose_pub;
  if (publish_vision_pose) {
    vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "/mavros/vision_pose/pose", 30);
  }
  ros::Publisher setpoint_pub =
      nh.advertise<geometry_msgs::PoseStamped>(
          "/mavros/setpoint_position/local", 30);
  SetpointStreamer setpoint_streamer(setpoint_pub, 30.0);

  ros::ServiceClient arming_client =
      nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  ros::ServiceClient mode_client =
      nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

  ros::Rate rate(30.0);

  ROS_INFO("Waiting for PX4 and LiDAR odometry on %s...", odom_topic.c_str());
  while (ros::ok()) {
    ros::spinOnce();
    const ros::Time now = ros::Time::now();

    if (publish_vision_pose && odometryIsFresh(now, odom_timeout)) {
      geometry_msgs::PoseStamped vision_pose = latest_lidar_pose;
      vision_pose.header.stamp = now;
      vision_pose_pub.publish(vision_pose);
    }

    if (current_state.connected && odometryIsFresh(now, odom_timeout)) {
      break;
    }

    ROS_INFO_THROTTLE(2.0, "PX4 connected=%s, LiDAR odometry fresh=%s",
                      current_state.connected ? "true" : "false",
                      odometryIsFresh(now, odom_timeout) ? "true" : "false");
    rate.sleep();
  }

  if (!ros::ok()) {
    return 0;
  }

  mavros_msgs::SetMode offboard_mode;
  offboard_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_command;
  arm_command.request.value = true;

  const geometry_msgs::PoseStamped takeoff_target =
      makeHoverTarget(hover_height);
  geometry_msgs::PoseStamped hold_target = takeoff_target;
  geometry_msgs::PoseStamped active_target = takeoff_target;

  FlightPhase phase = FlightPhase::ARMING;
  ros::Time last_arm_request;
  ros::Time last_mode_request;
  ros::Time prestream_started;
  ros::Time hover_stable_since;
  uint32_t active_trajectory_id = 0;

  ROS_INFO("Ready: ARM first, then pre-stream %.2f m setpoints for %.1f s, "
           "then enter OFFBOARD",
           hover_height, prestream_time);
  ROS_INFO("PX4 external vision publisher: %s",
           publish_vision_pose
               ? "internal /Odometry -> /mavros/vision_pose/pose bridge"
               : "external camera_pose_node expected");

  while (ros::ok()) {
    ros::spinOnce();
    const ros::Time now = ros::Time::now();

    if (!current_state.connected) {
      setpoint_streamer.setEnabled(false);
      ROS_ERROR_THROTTLE(1.0, "PX4 disconnected; no setpoints are being sent");
      rate.sleep();
      continue;
    }

    if (!odometryIsFresh(now, odom_timeout)) {
      setpoint_streamer.setEnabled(false);
      // Stop setpoints (and the optional internal vision stream) so PX4's
      // configured OFFBOARD-loss failsafe can take control.
      ROS_ERROR_THROTTLE(
          1.0, "LiDAR odometry timed out; stopping OFFBOARD setpoints");
      rate.sleep();
      continue;
    }

    if (publish_vision_pose) {
      geometry_msgs::PoseStamped vision_pose = latest_lidar_pose;
      vision_pose.header.stamp = now;
      vision_pose_pub.publish(vision_pose);
    }

    bool publish_setpoint = false;

    switch (phase) {
      case FlightPhase::ARMING: {
        if (current_state.armed) {
          phase = FlightPhase::PRESTREAM_SETPOINTS;
          prestream_started = now;
          ROS_INFO("Vehicle armed; starting continuous takeoff setpoint stream");
          break;
        }

        if (last_arm_request.isZero() ||
            now - last_arm_request >= ros::Duration(request_interval)) {
          if (arming_client.call(arm_command) && arm_command.response.success) {
            ROS_INFO("ARM command accepted; waiting for armed state");
          } else {
            ROS_WARN("ARM command rejected (result=%u); will retry",
                     static_cast<unsigned>(arm_command.response.result));
          }
          last_arm_request = now;
        }
        break;
      }

      case FlightPhase::PRESTREAM_SETPOINTS: {
        publish_setpoint = true;
        active_target = takeoff_target;

        if (!current_state.armed) {
          ROS_ERROR("Vehicle disarmed before takeoff; returning to ARMING");
          phase = FlightPhase::ARMING;
          last_arm_request = ros::Time();
        } else if (now - prestream_started >= ros::Duration(prestream_time)) {
          phase = FlightPhase::ENTERING_OFFBOARD;
          ROS_INFO("Setpoint pre-stream complete; requesting OFFBOARD");
        }
        break;
      }

      case FlightPhase::ENTERING_OFFBOARD: {
        publish_setpoint = true;
        active_target = takeoff_target;

        if (!current_state.armed) {
          ROS_ERROR("Vehicle disarmed before OFFBOARD; returning to ARMING");
          phase = FlightPhase::ARMING;
          last_arm_request = ros::Time();
          break;
        }

        if (current_state.mode == "OFFBOARD") {
          phase = FlightPhase::TAKING_OFF;
          hover_stable_since = ros::Time();
          ROS_INFO("OFFBOARD active; taking off to %.2f m", hover_height);
        } else if (last_mode_request.isZero() ||
                   now - last_mode_request >=
                       ros::Duration(request_interval)) {
          if (mode_client.call(offboard_mode) &&
              offboard_mode.response.mode_sent) {
            ROS_INFO("OFFBOARD request sent by MAVROS; waiting for mode state");
          } else {
            ROS_WARN("OFFBOARD request rejected; will retry");
          }
          last_mode_request = now;
        }
        break;
      }

      case FlightPhase::TAKING_OFF: {
        publish_setpoint = true;
        active_target = takeoff_target;

        const double error = positionError(latest_lidar_pose, takeoff_target);
        if (error <= hover_tolerance) {
          if (hover_stable_since.isZero()) {
            hover_stable_since = now;
          } else if (now - hover_stable_since >=
                     ros::Duration(hover_stable_time)) {
            phase = FlightPhase::WAITING_FOR_EGO;
            hold_target = takeoff_target;
            ROS_INFO("Takeoff complete; hovering at %.2f m and waiting for EGO "
                     "commands on %s",
                     hover_height, ego_cmd_topic.c_str());
          }
        } else {
          hover_stable_since = ros::Time();
        }
        break;
      }

      case FlightPhase::WAITING_FOR_EGO: {
        publish_setpoint = true;
        active_target = hold_target;

        if (egoCommandIsFresh(now, ego_timeout)) {
          active_target = egoCommandToTarget(
              latest_ego_command, ego_command_is_relative);
          active_trajectory_id = latest_ego_command.trajectory_id;
          phase = FlightPhase::TRACKING_EGO;
          ROS_INFO("Fresh EGO trajectory %u received; starting trajectory tracking",
                   static_cast<unsigned>(active_trajectory_id));
        } else {
          ROS_INFO_THROTTLE(2.0, "Hovering and waiting for EGO command on %s",
                            ego_cmd_topic.c_str());
        }
        break;
      }

      case FlightPhase::TRACKING_EGO: {
        publish_setpoint = true;

        if (egoCommandIsFresh(now, ego_timeout)) {
          active_target = egoCommandToTarget(
              latest_ego_command, ego_command_is_relative);
          if (latest_ego_command.trajectory_id != active_trajectory_id) {
            active_trajectory_id = latest_ego_command.trajectory_id;
            ROS_INFO("Switching to EGO trajectory %u",
                     static_cast<unsigned>(active_trajectory_id));
          }
        } else {
          hold_target = makeCurrentHoldTarget();
          active_target = hold_target;
          phase = FlightPhase::WAITING_FOR_EGO;
          ROS_WARN("EGO command timed out; holding current position and waiting "
                   "for a new trajectory");
        }
        break;
      }
    }

    // Once OFFBOARD flight has started, keep publishing the active target and
    // retry OFFBOARD if it is temporarily lost. Do not attempt to re-arm after
    // an in-flight disarm.
    const bool flight_started =
        phase == FlightPhase::TAKING_OFF ||
        phase == FlightPhase::WAITING_FOR_EGO ||
        phase == FlightPhase::TRACKING_EGO;

    if (flight_started && !current_state.armed) {
      ROS_FATAL("Vehicle disarmed after takeoff started; stopping controller");
      return 2;
    }

    if (flight_started && current_state.mode != "OFFBOARD" &&
        (last_mode_request.isZero() ||
         now - last_mode_request >= ros::Duration(request_interval))) {
      if (mode_client.call(offboard_mode) &&
          offboard_mode.response.mode_sent) {
        ROS_WARN("OFFBOARD was lost; re-entry request sent by MAVROS");
      } else {
        ROS_ERROR("OFFBOARD was lost; re-entry request rejected");
      }
      last_mode_request = now;
    }

    if (publish_setpoint) {
      setpoint_streamer.setTarget(active_target);
    }
    setpoint_streamer.setEnabled(publish_setpoint);

    ROS_INFO_THROTTLE(
        2.0, "phase=%s armed=%s mode=%s position=[%.2f %.2f %.2f]",
        phaseName(phase), current_state.armed ? "true" : "false",
        current_state.mode.c_str(), latest_lidar_pose.pose.position.x,
        latest_lidar_pose.pose.position.y,
        latest_lidar_pose.pose.position.z);

    rate.sleep();
  }

  (void)state_sub;
  (void)odom_sub;
  (void)ego_cmd_sub;
  return 0;
}
