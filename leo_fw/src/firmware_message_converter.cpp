#include <cmath>

#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"

#include "leo_msgs/Imu.h"
#include "leo_msgs/WheelOdom.h"
#include "leo_msgs/WheelStates.h"

static ros::Subscriber wheel_states_sub;
static ros::Publisher joint_states_pub;
static bool joint_states_advertised = false;

static ros::Subscriber wheel_odom_sub;
static ros::Publisher wheel_odom_pub;
static bool wheel_odom_advertised = false;

static ros::Subscriber imu_sub;
static ros::Publisher imu_pub;
static bool imu_advertised = false;

std::string robot_frame_id = "base_link";
std::string odom_frame_id = "odom";
std::string imu_frame_id = "imu";
std::vector<std::string> wheel_joint_names = {
    "wheel_FL_joint", "wheel_RL_joint", "wheel_FR_joint", "wheel_RR_joint"};
std::vector<double> wheel_odom_twist_covariance_diagonal = {0.0001, 0.0, 0.0,
                                                            0.0,    0.0, 0.001};
std::vector<double> imu_angular_velocity_covariance_diagonal = {
    0.000001, 0.000001, 0.00001};
std::vector<double> imu_linear_acceleration_covariance_diagonal = {0.001, 0.001,
                                                                   0.001};
std::string tf_frame_prefix = "";

void load_parameters(ros::NodeHandle &pnh) {
  pnh.getParam("robot_frame_id", robot_frame_id);
  pnh.getParam("odom_frame_id", odom_frame_id);
  pnh.getParam("imu_frame_id", imu_frame_id);
  pnh.getParam("wheel_joint_names", wheel_joint_names);
  pnh.getParam("wheel_odom_twist_covariance_diagonal",
               wheel_odom_twist_covariance_diagonal);
  pnh.getParam("imu_angular_velocity_covariance_diagonal",
               imu_angular_velocity_covariance_diagonal);
  pnh.getParam("imu_linear_acceleration_covariance_diagonal",
               imu_linear_acceleration_covariance_diagonal);
  pnh.getParam("tf_frame_prefix", tf_frame_prefix);

  robot_frame_id = tf_frame_prefix + robot_frame_id;
  odom_frame_id = tf_frame_prefix + odom_frame_id;
  imu_frame_id = tf_frame_prefix + imu_frame_id;
}

void wheel_states_callback(const leo_msgs::WheelStatesPtr &msg) {
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

void wheel_odom_callback(const leo_msgs::WheelOdomPtr &msg) {
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

  for (int i = 0; i < 6; i++)
    wheel_odom.twist.covariance[i * 7] =
        wheel_odom_twist_covariance_diagonal[i];

  wheel_odom_pub.publish(wheel_odom);
}
void imu_callback(const leo_msgs::ImuPtr &msg) {
  sensor_msgs::Imu imu;
  imu.header.frame_id = imu_frame_id;
  imu.header.stamp = msg->stamp;
  imu.angular_velocity.x = msg->gyro_x;
  imu.angular_velocity.y = msg->gyro_y;
  imu.angular_velocity.z = msg->gyro_z;
  imu.linear_acceleration.x = msg->accel_x;
  imu.linear_acceleration.y = msg->accel_y;
  imu.linear_acceleration.z = msg->accel_z;

  for (int i = 0; i < 3; i++) {
    imu.angular_velocity_covariance[i * 4] =
        imu_angular_velocity_covariance_diagonal[i];
    imu.linear_acceleration_covariance[i * 4] =
        imu_linear_acceleration_covariance_diagonal[i];
  }

  imu_pub.publish(imu);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "firmware_message_converter");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  load_parameters(pnh);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  const std::string wheel_states_topic =
      ros::names::resolve("firmware/wheel_states");
  const std::string wheel_odom_topic =
      ros::names::resolve("firmware/wheel_odom");
  const std::string imu_topic = ros::names::resolve("firmware/imu");

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
          "Shutting down wheel_odom_with_covariance publisher.");
      wheel_odom_sub.shutdown();
      wheel_odom_pub.shutdown();
      wheel_odom_advertised = false;
    }
    if (imu_advertised && imu_sub.getNumPublishers() == 0) {
      ROS_INFO(
          "firmware/imu topic no longer has any publishers. "
          "Shutting down imu/data_raw publisher.");
      imu_sub.shutdown();
      imu_pub.shutdown();
      imu_advertised = false;
    }

    rate.sleep();

    // Scan topics
    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);

    for (auto &topic : topics) {
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
      if (!imu_advertised && topic.name == imu_topic) {
        ROS_INFO(
            "Detected firmware/imu topic advertised. "
            "Advertising imu/data_raw topic.");
        imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
        imu_sub = nh.subscribe("firmware/imu", 5, imu_callback);
        imu_advertised = true;
      }
    }

    rate.sleep();
  }
}