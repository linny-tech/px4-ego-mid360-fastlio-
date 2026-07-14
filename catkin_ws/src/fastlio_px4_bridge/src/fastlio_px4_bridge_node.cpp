#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace {

bool finiteQuaternion(const geometry_msgs::Quaternion &q)
{
  return std::isfinite(q.x) && std::isfinite(q.y) &&
         std::isfinite(q.z) && std::isfinite(q.w);
}

geometry_msgs::Quaternion toMessage(const tf2::Quaternion &q)
{
  geometry_msgs::Quaternion result;
  result.x = q.x();
  result.y = q.y();
  result.z = q.z();
  result.w = q.w();
  return result;
}

tf2::Matrix3x3 opticalToEnuMatrix()
{
  // Raw optical convention: X right, Y down, Z forward.
  // ROS ENU/FLU convention: X forward, Y left, Z up.
  // Therefore: enu_x=raw_z, enu_y=-raw_x, enu_z=-raw_y.
  return tf2::Matrix3x3(
      0.0,  0.0,  1.0,
     -1.0,  0.0,  0.0,
      0.0, -1.0,  0.0);
}

bool validRotation(const tf2::Matrix3x3 &matrix)
{
  const tf2::Matrix3x3 should_be_identity = matrix * matrix.transpose();
  double max_error = 0.0;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      const double expected = row == column ? 1.0 : 0.0;
      max_error = std::max(
          max_error,
          std::fabs(should_be_identity[row][column] - expected));
    }
  }
  return max_error < 1e-6 && std::fabs(matrix.determinant() - 1.0) < 1e-6;
}

class FastlioPx4Bridge {
 public:
  FastlioPx4Bridge()
      : nh_(), private_nh_("~"), initialized_(false), have_pose_(false)
  {
    private_nh_.param<std::string>(
        "raw_odom_topic", raw_odom_topic_, "/Odometry");
    private_nh_.param<std::string>(
        "enu_odom_topic", enu_odom_topic_, "/Odometry_enu");
    private_nh_.param<std::string>(
        "vision_pose_topic", vision_pose_topic_, "/mavros/vision_pose/pose");
    private_nh_.param<std::string>(
        "axis_mode", axis_mode_, "optical_to_enu");
    private_nh_.param("zero_origin", zero_origin_, true);
    private_nh_.param("zero_initial_yaw", zero_initial_yaw_, true);
    private_nh_.param("vision_rate", vision_rate_, 30.0);
    private_nh_.param("odom_timeout", odom_timeout_, 0.5);

    if (vision_rate_ <= 2.0) {
      ROS_WARN("vision_rate must exceed 2 Hz; using 30 Hz");
      vision_rate_ = 30.0;
    }
    if (odom_timeout_ <= 0.0) {
      odom_timeout_ = 0.5;
    }

    axis_rotation_ = axisMatrixFromParameters();
    if (!validRotation(axis_rotation_)) {
      ROS_FATAL("axis_matrix is not a right-handed orthonormal rotation");
      throw std::runtime_error("invalid axis rotation");
    }

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(enu_odom_topic_, 50);
    vision_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>(vision_pose_topic_, 30);
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        raw_odom_topic_, 100, &FastlioPx4Bridge::odomCallback, this);
    vision_timer_ = nh_.createWallTimer(
        ros::WallDuration(1.0 / vision_rate_),
        &FastlioPx4Bridge::visionTimerCallback, this);

    ROS_INFO("Fast-LIO axis bridge: %s -> %s -> %s",
             raw_odom_topic_.c_str(), enu_odom_topic_.c_str(),
             vision_pose_topic_.c_str());
    ROS_INFO("axis_mode=%s, zero_origin=%s, zero_initial_yaw=%s",
             axis_mode_.c_str(), zero_origin_ ? "true" : "false",
             zero_initial_yaw_ ? "true" : "false");
    ROS_INFO("Configured mapping: ENU[x y z] = [raw_z -raw_x -raw_y] "
             "for optical_to_enu mode");
  }

 private:
  tf2::Matrix3x3 axisMatrixFromParameters()
  {
    std::vector<double> values;
    if (private_nh_.getParam("axis_matrix", values)) {
      if (values.size() != 9) {
        throw std::runtime_error("axis_matrix must contain exactly 9 values");
      }
      axis_mode_ = "custom";
      return tf2::Matrix3x3(
          values[0], values[1], values[2],
          values[3], values[4], values[5],
          values[6], values[7], values[8]);
    }

    if (axis_mode_ == "identity") {
      return tf2::Matrix3x3::getIdentity();
    }
    if (axis_mode_ == "optical_to_enu") {
      return opticalToEnuMatrix();
    }

    throw std::runtime_error(
        "axis_mode must be identity or optical_to_enu");
  }

  void initializeFrame(const nav_msgs::Odometry &message,
                       const tf2::Quaternion &raw_attitude)
  {
    raw_origin_.setValue(
        message.pose.pose.position.x,
        message.pose.pose.position.y,
        message.pose.pose.position.z);

    tf2::Quaternion axis_quaternion;
    axis_rotation_.getRotation(axis_quaternion);
    axis_quaternion.normalize();
    output_from_raw_ = axis_quaternion;

    if (zero_initial_yaw_) {
      tf2::Quaternion aligned_attitude = axis_quaternion * raw_attitude;
      aligned_attitude.normalize();
      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      tf2::Matrix3x3(aligned_attitude).getRPY(roll, pitch, yaw);

      tf2::Quaternion remove_initial_yaw;
      remove_initial_yaw.setRPY(0.0, 0.0, -yaw);
      remove_initial_yaw.normalize();
      output_from_raw_ = remove_initial_yaw * axis_quaternion;
      output_from_raw_.normalize();
    }

    initialized_ = true;
    ROS_INFO("Fast-LIO ENU origin fixed at raw [%.3f %.3f %.3f]",
             raw_origin_.x(), raw_origin_.y(), raw_origin_.z());
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &message)
  {
    const geometry_msgs::Point &position = message->pose.pose.position;
    const geometry_msgs::Quaternion &orientation =
        message->pose.pose.orientation;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z) || !finiteQuaternion(orientation)) {
      ROS_WARN_THROTTLE(1.0, "Ignoring non-finite Fast-LIO odometry");
      return;
    }

    tf2::Quaternion raw_attitude(
        orientation.x, orientation.y, orientation.z, orientation.w);
    if (raw_attitude.length2() < 1e-8) {
      ROS_WARN_THROTTLE(1.0, "Ignoring invalid Fast-LIO quaternion");
      return;
    }
    raw_attitude.normalize();

    if (!initialized_) {
      initializeFrame(*message, raw_attitude);
    }

    const tf2::Vector3 raw_position(position.x, position.y, position.z);
    const tf2::Vector3 origin = zero_origin_ ? raw_origin_ : tf2::Vector3(0, 0, 0);
    const tf2::Vector3 output_position =
        tf2::quatRotate(output_from_raw_, raw_position - origin);

    tf2::Quaternion output_attitude = output_from_raw_ * raw_attitude;
    output_attitude.normalize();

    nav_msgs::Odometry output = *message;
    output.header.frame_id = "map";
    output.child_frame_id = "base_link";
    output.pose.pose.position.x = output_position.x();
    output.pose.pose.position.y = output_position.y();
    output.pose.pose.position.z = output_position.z();
    output.pose.pose.orientation = toMessage(output_attitude);

    // FAST-LIO's published twist is normally zero, but rotate it consistently
    // when a modified build supplies world-frame velocity.
    const tf2::Vector3 raw_linear(
        message->twist.twist.linear.x,
        message->twist.twist.linear.y,
        message->twist.twist.linear.z);
    const tf2::Vector3 output_linear =
        tf2::quatRotate(output_from_raw_, raw_linear);
    output.twist.twist.linear.x = output_linear.x();
    output.twist.twist.linear.y = output_linear.y();
    output.twist.twist.linear.z = output_linear.z();

    geometry_msgs::PoseStamped vision;
    vision.header = output.header;
    vision.pose = output.pose.pose;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_vision_pose_ = vision;
      last_odom_wall_time_ = ros::WallTime::now();
      have_pose_ = true;
    }

    odom_pub_.publish(output);
  }

  void visionTimerCallback(const ros::WallTimerEvent &)
  {
    geometry_msgs::PoseStamped vision;
    ros::WallTime last_received;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_pose_) {
        return;
      }
      vision = latest_vision_pose_;
      last_received = last_odom_wall_time_;
    }

    const double age = (ros::WallTime::now() - last_received).toSec();
    if (age > odom_timeout_) {
      ROS_ERROR_THROTTLE(
          1.0, "Raw Fast-LIO odometry timed out; vision publishing stopped");
      return;
    }

    vision.header.stamp = ros::Time::now();
    vision_pub_.publish(vision);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odom_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher vision_pub_;
  ros::WallTimer vision_timer_;

  std::string raw_odom_topic_;
  std::string enu_odom_topic_;
  std::string vision_pose_topic_;
  std::string axis_mode_;
  bool zero_origin_;
  bool zero_initial_yaw_;
  double vision_rate_;
  double odom_timeout_;

  tf2::Matrix3x3 axis_rotation_;
  tf2::Quaternion output_from_raw_;
  tf2::Vector3 raw_origin_;
  bool initialized_;

  std::mutex mutex_;
  geometry_msgs::PoseStamped latest_vision_pose_;
  ros::WallTime last_odom_wall_time_;
  bool have_pose_;
};

}  // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fastlio_px4_bridge_node");
  try {
    FastlioPx4Bridge bridge;
    ros::spin();
  } catch (const std::exception &error) {
    ROS_FATAL("Failed to start Fast-LIO PX4 bridge: %s", error.what());
    return 1;
  }
  return 0;
}
