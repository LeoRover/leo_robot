#include <cmath>
#include <cstdlib>
#include <fstream>

#include "ros/ros.h"
#include "yaml-cpp/yaml.h"

#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "std_srvs/Trigger.h"

#include "leo_msgs/Imu.h"
#include "leo_msgs/WheelOdom.h"
#include "leo_msgs/WheelOdomMecanum.h"
#include "leo_msgs/WheelStates.h"

#include "leo_msgs/SetImuCalibration.h"

constexpr double PI = 3.141592653;

static ros::Subscriber wheel_states_sub;
static ros::Publisher joint_states_pub;
static bool joint_states_advertised = false;

static ros::Subscriber wheel_odom_sub;
static ros::Publisher wheel_odom_pub;
static bool wheel_odom_advertised = false;

static bool wheel_odom_mecanum_advertised = false;
static ros::Subscriber wheel_odom_mecanum_sub;

static ros::Subscriber imu_sub;
static ros::Publisher imu_pub;
static bool imu_advertised = false;

static ros::Publisher odom_merged_pub;
static ros::Timer odom_merged_timer;
static geometry_msgs::Point odom_merged_position;
static double odom_merged_yaw;
static bool odom_merged_advertised = false;
static double velocity_linear_x = 0;
static double velocity_linear_y = 0;
static double velocity_angular_z = 0;
static ros::ServiceClient odom_reset_client;
static ros::ServiceServer reset_odometry_service;

ros::ServiceServer set_imu_calibration_service;

std::string robot_frame_id = "base_link";
std::string odom_frame_id = "odom";
std::string imu_frame_id = "imu";
std::vector<std::string> wheel_joint_names = {
    "wheel_FL_joint", "wheel_RL_joint", "wheel_FR_joint", "wheel_RR_joint"};
std::vector<double> wheel_odom_twist_covariance_diagonal = {0.0001, 0.0, 0.0,
                                                            0.0,    0.0, 0.001};
std::vector<double> wheel_odom_mecanum_twist_covariance_diagonal = {
    0.0001, 0.0001, 0.0, 0.0, 0.0, 0.001};

std::vector<double> imu_angular_velocity_covariance_diagonal = {
    0.000001, 0.000001, 0.00001};
std::vector<double> imu_linear_acceleration_covariance_diagonal = {0.001, 0.001,
                                                                   0.001};
std::string tf_frame_prefix = "";

std::vector<float> imu_calibration_bias = {0.0, 0.0, 0.0};
std::string calib_file_path = "";

void load_parameters(ros::NodeHandle& pnh) {
  pnh.getParam("robot_frame_id", robot_frame_id);
  pnh.getParam("odom_frame_id", odom_frame_id);
  pnh.getParam("imu_frame_id", imu_frame_id);
  pnh.getParam("wheel_joint_names", wheel_joint_names);
  pnh.getParam("wheel_odom_twist_covariance_diagonal",
               wheel_odom_twist_covariance_diagonal);
  pnh.getParam("wheel_odom_mecanum_twist_covariance_diagonal",
               wheel_odom_mecanum_twist_covariance_diagonal);
  pnh.getParam("imu_angular_velocity_covariance_diagonal",
               imu_angular_velocity_covariance_diagonal);
  pnh.getParam("imu_linear_acceleration_covariance_diagonal",
               imu_linear_acceleration_covariance_diagonal);
  pnh.getParam("tf_frame_prefix", tf_frame_prefix);

  robot_frame_id = tf_frame_prefix + robot_frame_id;
  odom_frame_id = tf_frame_prefix + odom_frame_id;
  imu_frame_id = tf_frame_prefix + imu_frame_id;
}

void wheel_states_callback(const leo_msgs::WheelStatesPtr& msg) {
  sensor_msgs::JointState joint_states;
  joint_states.header.stamp = msg->stamp;
  joint_states.name = wheel_joint_names;
  joint_states.position =
      std::vector<double>(msg->position.begin(), msg->position.end());
  joint_states.velocity =
      std::vector<double>(msg->velocity.begin(), msg->velocity.end());
  joint_states.effort =
      std::vector<double>(msg->torque.begin(), msg->torque.end());

  joint_states_pub.publish(joint_states);
}

void wheel_odom_callback(const leo_msgs::WheelOdomPtr& msg) {
  nav_msgs::Odometry wheel_odom;
  wheel_odom.header.frame_id = odom_frame_id;
  wheel_odom.child_frame_id = robot_frame_id;
  wheel_odom.header.stamp = msg->stamp;
  wheel_odom.twist.twist.linear.x = msg->velocity_lin;
  wheel_odom.twist.twist.angular.z = msg->velocity_ang;
  wheel_odom.pose.pose.position.x = msg->pose_x;
  wheel_odom.pose.pose.position.y = msg->pose_y;
  wheel_odom.pose.pose.orientation.z = std::sin(msg->pose_yaw * 0.5F);
  wheel_odom.pose.pose.orientation.w = std::cos(msg->pose_yaw * 0.5F);

  velocity_linear_x = msg->velocity_lin;

  for (int i = 0; i < 6; i++)
    wheel_odom.twist.covariance[i * 7] =
        wheel_odom_twist_covariance_diagonal[i];

  wheel_odom_pub.publish(wheel_odom);
}

void imu_callback(const leo_msgs::ImuPtr& msg) {
  sensor_msgs::Imu imu;
  imu.header.frame_id = imu_frame_id;
  imu.header.stamp = msg->stamp;
  imu.angular_velocity.x = msg->gyro_x + imu_calibration_bias[0];
  imu.angular_velocity.y = msg->gyro_y + imu_calibration_bias[1];
  imu.angular_velocity.z = msg->gyro_z + imu_calibration_bias[2];
  imu.linear_acceleration.x = msg->accel_x;
  imu.linear_acceleration.y = msg->accel_y;
  imu.linear_acceleration.z = msg->accel_z;

  velocity_angular_z = imu.angular_velocity.z;

  for (int i = 0; i < 3; i++) {
    imu.angular_velocity_covariance[i * 4] =
        imu_angular_velocity_covariance_diagonal[i];
    imu.linear_acceleration_covariance[i * 4] =
        imu_linear_acceleration_covariance_diagonal[i];
  }

  imu_pub.publish(imu);
}

void mecanum_odom_callback(const leo_msgs::WheelOdomMecanumPtr& msg) {
  nav_msgs::Odometry wheel_odom;
  wheel_odom.header.frame_id = odom_frame_id;
  wheel_odom.child_frame_id = robot_frame_id;
  wheel_odom.header.stamp = msg->stamp;
  wheel_odom.twist.twist.linear.x = msg->velocity_lin_x;
  wheel_odom.twist.twist.linear.y = msg->velocity_lin_y;
  wheel_odom.twist.twist.angular.z = msg->velocity_ang;
  wheel_odom.pose.pose.position.x = msg->pose_x;
  wheel_odom.pose.pose.position.y = msg->pose_y;
  wheel_odom.pose.pose.orientation.z = std::sin(msg->pose_yaw * 0.5F);
  wheel_odom.pose.pose.orientation.w = std::cos(msg->pose_yaw * 0.5F);

  velocity_linear_x = msg->velocity_lin_x;
  velocity_linear_y = msg->velocity_lin_y;

  for (int i = 0; i < 6; i++)
    wheel_odom.twist.covariance[i * 7] =
        wheel_odom_mecanum_twist_covariance_diagonal[i];

  wheel_odom_pub.publish(wheel_odom);
}

void merge_odometry_callback(const ros::TimerEvent& events) {
  nav_msgs::Odometry merged_odom;
  merged_odom.header.frame_id = odom_frame_id;
  merged_odom.child_frame_id = robot_frame_id;
  merged_odom.header.stamp = ros::Time::now();
  merged_odom.twist.twist.linear.x = velocity_linear_x;
  merged_odom.twist.twist.linear.y = velocity_linear_y;
  merged_odom.twist.twist.angular.z = velocity_angular_z;

  const double move_x = velocity_linear_x * std::cos(odom_merged_yaw) -
                        velocity_linear_y * std::sin(odom_merged_yaw);
  const double move_y = velocity_linear_x * std::sin(odom_merged_yaw) +
                        velocity_linear_y * std::cos(odom_merged_yaw);

  odom_merged_position.x += move_x * 0.01;
  odom_merged_position.y += move_y * 0.01;

  odom_merged_yaw += velocity_angular_z * 0.01;

  if (odom_merged_yaw > 2.0 * PI)
    odom_merged_yaw -= 2.0 * PI;
  else if (odom_merged_yaw < 0.0)
    odom_merged_yaw += 2.0 * PI;

  merged_odom.pose.pose.position.x = odom_merged_position.x;
  merged_odom.pose.pose.position.y = odom_merged_position.y;
  merged_odom.pose.pose.orientation.z = std::sin(odom_merged_yaw * 0.5);
  merged_odom.pose.pose.orientation.w = std::cos(odom_merged_yaw * 0.5);

  std::vector<double>* merged_covariance;

  if (wheel_odom_mecanum_advertised)
    merged_covariance = &wheel_odom_mecanum_twist_covariance_diagonal;
  else
    merged_covariance = &wheel_odom_twist_covariance_diagonal;

  for (int i = 0; i < 5; i++)
    merged_odom.twist.covariance[i * 7] = (*merged_covariance)[i];
  merged_odom.twist.covariance[35] =
      imu_angular_velocity_covariance_diagonal[2];

  odom_merged_pub.publish(merged_odom);
}

void load_yaml_bias() {
  YAML::Node node;
  try {
    node = YAML::LoadFile(calib_file_path);

    if (node["gyro_bias_x"])
      imu_calibration_bias[0] = node["gyro_bias_x"].as<float>();

    if (node["gyro_bias_y"])
      imu_calibration_bias[1] = node["gyro_bias_y"].as<float>();

    if (node["gyro_bias_z"])
      imu_calibration_bias[2] = node["gyro_bias_z"].as<float>();

  } catch (YAML::BadFile& e) {
    std::cerr << "Calibration file doesn't exist.\n";
    std::cerr << "Creating calibration file with default gyrometer bias.\n";

    node["gyro_bias_x"] = imu_calibration_bias[0];
    node["gyro_bias_y"] = imu_calibration_bias[1];
    node["gyro_bias_z"] = imu_calibration_bias[2];

    std::ofstream fout(calib_file_path);
    fout << node;
  }
}

std::string get_calib_path() {
  std::string ros_home;
  char* ros_home_env;
  if (ros_home_env = std::getenv("ROS_HOME")) {
    ros_home = ros_home_env;
  } else if (ros_home_env = std::getenv("HOME")) {
    ros_home = ros_home_env;
    ros_home += "/.ros";
  }

  return ros_home + "/imu_calibration.yaml";
}

bool set_imu_calibration_callback(leo_msgs::SetImuCalibrationRequest& req,
                                  leo_msgs::SetImuCalibrationResponse& res) {
  ROS_INFO("SetImuCalibration request for: [ %f, %f, %f]", req.gyro_bias_x,
           req.gyro_bias_y, req.gyro_bias_z);

  YAML::Node node = YAML::LoadFile(calib_file_path);
  node["gyro_bias_x"] = imu_calibration_bias[0] = req.gyro_bias_x;
  node["gyro_bias_y"] = imu_calibration_bias[1] = req.gyro_bias_y;
  node["gyro_bias_z"] = imu_calibration_bias[2] = req.gyro_bias_z;
  std::ofstream fout(calib_file_path);
  fout << node;

  res.success = true;
  return true;
}

bool reset_odometry_callback(std_srvs::TriggerRequest& req,
                             std_srvs::TriggerResponse& res) {
  odom_merged_position.x = 0.0;
  odom_merged_position.y = 0.0;
  odom_merged_yaw = 0.0;
  if (odom_reset_client.call(req, res)) {
    res.success = true;
    return true;
  } else {
    res.success = false;
    return false;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "firmware_message_converter");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  load_parameters(pnh);
  calib_file_path = get_calib_path();
  load_yaml_bias();

  ros::AsyncSpinner spinner(4);
  spinner.start();

  const std::string wheel_states_topic =
      ros::names::resolve("firmware/wheel_states");
  const std::string wheel_odom_topic =
      ros::names::resolve("firmware/wheel_odom");
  const std::string wheel_odom_mecanum_topic =
      ros::names::resolve("firmware/wheel_odom_mecanum");
  const std::string imu_topic = ros::names::resolve("firmware/imu");

  set_imu_calibration_service =
      nh.advertiseService("set_imu_calibration", set_imu_calibration_callback);
  odom_reset_client =
      nh.serviceClient<std_srvs::Trigger>("firmware/reset_odometry");
  reset_odometry_service =
      nh.advertiseService("reset_odometry", &reset_odometry_callback);

  ros::Rate rate(2);
  while (ros::ok()) {
    // Shutdown inactive topics
    if (joint_states_advertised && wheel_states_sub.getNumPublishers() == 0) {
      ROS_INFO(
          "firmware/wheel_states topic no longer has any publishers. "
          "Shutting down joint_states publisher.");
      wheel_states_sub.shutdown();
      joint_states_pub.shutdown();
      joint_states_advertised = false;
    }
    if (wheel_odom_advertised && wheel_odom_sub.getNumPublishers() == 0) {
      ROS_INFO(
          "firmware/wheel_odom topic no longer has any publishers. "
          "Shutting down wheel_odom_with_covariance and odometry_merged "
          "publishers.");
      wheel_odom_sub.shutdown();
      wheel_odom_pub.shutdown();
      odom_merged_pub.shutdown();
      odom_merged_timer.stop();
      wheel_odom_advertised = false;
      odom_merged_advertised = false;
    }
    if (wheel_odom_mecanum_advertised &&
        wheel_odom_mecanum_sub.getNumPublishers() == 0) {
      ROS_INFO(
          "firmware/wheel_odom_mecanum topic no longer has any publishers. "
          "Shutting down wheel_odom_with_covariance and odometry_merged "
          "publishers.");
      wheel_odom_mecanum_sub.shutdown();
      wheel_odom_pub.shutdown();
      odom_merged_pub.shutdown();
      odom_merged_timer.stop();
      wheel_odom_mecanum_advertised = false;
      odom_merged_advertised = false;
    }
    if (imu_advertised && imu_sub.getNumPublishers() == 0) {
      ROS_INFO(
          "firmware/imu topic no longer has any publishers. "
          "Shutting down imu/data_raw and odometry_merged publishers.");
      imu_sub.shutdown();
      imu_pub.shutdown();
      odom_merged_pub.shutdown();
      odom_merged_timer.stop();
      imu_advertised = false;
      odom_merged_advertised = false;
    }

    rate.sleep();

    // Scan topics
    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);

    for (auto& topic : topics) {
      if (!joint_states_advertised && topic.name == wheel_states_topic) {
        ROS_INFO(
            "Detected firmware/wheel_states topic advertised. "
            "Advertising joint_states topic.");
        joint_states_pub =
            nh.advertise<sensor_msgs::JointState>("joint_states", 10);
        wheel_states_sub =
            nh.subscribe("firmware/wheel_states", 5, wheel_states_callback);
        joint_states_advertised = true;
      }
      if (!wheel_odom_advertised && topic.name == wheel_odom_topic) {
        ROS_INFO(
            "Detected firmware/wheel_odom topic advertised. "
            "Advertising wheel_odom_with_covariance topic.");
        wheel_odom_pub =
            nh.advertise<nav_msgs::Odometry>("wheel_odom_with_covariance", 10);
        wheel_odom_sub =
            nh.subscribe("firmware/wheel_odom", 5, wheel_odom_callback);
        wheel_odom_advertised = true;
      }
      if (!wheel_odom_mecanum_advertised &&
          topic.name == wheel_odom_mecanum_topic) {
        ROS_INFO(
            "Detected firmware/wheel_odom_mecanum topic advertised. "
            "Advertising wheel_odom_with_covariance topic.");
        wheel_odom_pub =
            nh.advertise<nav_msgs::Odometry>("wheel_odom_with_covariance", 10);
        wheel_odom_mecanum_sub = nh.subscribe("firmware/wheel_odom_mecanum", 5,
                                              mecanum_odom_callback);
        wheel_odom_mecanum_advertised = true;
      }
      if (!imu_advertised && topic.name == imu_topic) {
        ROS_INFO(
            "Detected firmware/imu topic advertised. "
            "Advertising imu/data_raw topic.");
        imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
        imu_sub = nh.subscribe("firmware/imu", 5, imu_callback);
        imu_advertised = true;
      }
      if (!odom_merged_advertised && imu_advertised &&
          (wheel_odom_advertised || wheel_odom_mecanum_advertised)) {
        ROS_INFO(
            "Both firmware/imu and (firmware/wheel_odom or "
            "firmware/wheel_odom_mecanum) topics are advertised. "
            "Advertising odometry_merged topic.");
        odom_merged_pub =
            nh.advertise<nav_msgs::Odometry>("odometry_merged", 10);
        odom_merged_timer =
            nh.createTimer(ros::Duration(0.01), merge_odometry_callback);
        odom_merged_advertised = true;
      }
    }

    rate.sleep();
  }
}