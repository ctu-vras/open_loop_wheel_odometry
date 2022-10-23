/**
 * \file
 * \brief A generalized node that receives cmd_vel commands and integrates them via an open-loop odometry model.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <array>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include <cras_cpp_common/node_utils.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/message_traits.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <topic_tools/shape_shifter.h>

#include <open_loop_wheel_odometry/open_loop_wheel_odometry.h>

namespace open_loop_wheel_odometry
{

/**
 * \brief This function either returns a string, or a notice about child frame ID being autodetected.
 * \param[in] value The string.
 * \return Either the input string or the notice.
 */
std::string baseLinkFrameToStr(const std::string& value)
{
  if (value.empty())
  {
    return "<Autodetect from header.frame_id of the cmd_vel messages>";
  }
  return value;
}

OpenLoopWheelOdom::~OpenLoopWheelOdom() // NOLINT(modernize-use-equals-default)
{
}

OpenLoopWheelOdom::OpenLoopWheelOdom(
  ros::NodeHandle nodeHandle,
  ros::NodeHandle privateNodeHandle, // NOLINT(performance-unnecessary-value-param)
  std::unique_ptr<Odometry> odom)
  : odom(std::move(odom))
{
  if (!ros::Time::isValid())
  {
    ROS_INFO("Waiting for valid ROS time.");
    ros::Time::waitForValid();
  }

  auto privateParams = cras::nodeParams(privateNodeHandle);

  const auto odomFrame = privateParams->getParam("odom_frame", "odom_cmd_vel");
  cras::GetParamOptions<std::string> opts;
  opts.resultToStr = &baseLinkFrameToStr;
  auto baseLinkFrame = privateParams->getParam("base_link_frame", "", "", opts);  // autodetect from cmd_vel

  this->initialPoseCovDiag = privateParams->getParam("initial_pose_covariance_diagonal", std::array<double, 6>(
    {1e-6, 1e-6, 1e-6, M_PI * M_PI, M_PI * M_PI, 1e-6}), "(m/s)^2 or (rad/s)^2");
  const auto twistCovDiag = privateParams->getParam("twist_covariance_diagonal", std::array<double, 6>(
    {1e-2, 1e-4, 1e-1, M_PI * M_PI, M_PI * M_PI, 0.5}), "(m/s)^2 or (rad/s)^2");

  // Prepare the odometry message
  this->odomMsg.header.frame_id = odomFrame;
  this->odomMsg.child_frame_id = baseLinkFrame;
  for (size_t i = 0; i < 6; ++i)
  {
    this->odomMsg.pose.covariance[i + i * 6] = this->initialPoseCovDiag[i];
  }
  for (size_t i = 0; i < 6; ++i)
  {
    this->odomMsg.twist.covariance[i + i * 6] = twistCovDiag[i];
  }

  // Set up publishers and subscribers
  this->pub = nodeHandle.template advertise<nav_msgs::Odometry>("odom_cmd_vel", 10);
  this->sub = nodeHandle.subscribe("cmd_vel_out", 10, &OpenLoopWheelOdom::onCmd, this);
  this->resetSub = nodeHandle.subscribe("reset", 1, &OpenLoopWheelOdom::onReset, this);
}

void OpenLoopWheelOdom::onCmd(const topic_tools::ShapeShifter& msg)
{
  if (msg.getDataType() == ros::message_traits::DataType<geometry_msgs::TwistStamped>::value())
  {
    const auto stampedMsg = msg.instantiate<geometry_msgs::TwistStamped>();
    if (this->odomMsg.child_frame_id.empty())
    {
      if (stampedMsg->header.frame_id.empty())
      {
        ROS_ERROR("The node is configured to autodetect base link frame ID from incoming cmd_vel messages, but these "
                  "come with an empty frame_id. Defaulting base_link_frame to 'base_link'");
        this->odomMsg.child_frame_id = "base_link";
      }
      else
      {
        this->odomMsg.child_frame_id = stampedMsg->header.frame_id;
      }
    }
    this->update(stampedMsg->twist, stampedMsg->header.stamp.isZero() ? ros::Time::now() : stampedMsg->header.stamp);
  }
  else if (msg.getDataType() == ros::message_traits::DataType<geometry_msgs::Twist>::value())
  {
    if (this->odomMsg.child_frame_id.empty())
    {
      ROS_ERROR("The node is configured to autodetect base link frame ID from incoming cmd_vel messages, but these "
                "are not stamped. Defaulting base_link_frame to 'base_link'");
      this->odomMsg.child_frame_id = "base_link";
    }
    this->update(*msg.instantiate<geometry_msgs::Twist>(), ros::Time::now());
  }
  else
  {
    ROS_ERROR("Received an unsupported message type %s", msg.getDataType().c_str());
  }
}

void OpenLoopWheelOdom::update(const geometry_msgs::Twist& msg, const ros::Time& stamp)
{
  if (this->initialized && ros::Time::isSimTime() && ros::Time::now() + ros::Duration(3) < this->lastTime)
  {
    ROS_INFO("ROS time jumped back.");
    this->reset();
  }

  if (!this->initialized)
  {
    this->odom->init(stamp);
    this->lastTime = ros::Time::now();
    this->lastMeasurementTime = stamp;
    this->lastMsg = msg;
    this->initialized = true;
    return;
  }

  // Perform the open-loop update of the odometry
  this->odom->updateOpenLoop(this->lastMsg.linear.x, this->lastMsg.angular.z, stamp);

  this->odomMsg.header.stamp = stamp;
  this->odomMsg.twist.twist.linear.x = this->odom->getLinear();
  this->odomMsg.twist.twist.angular.z = this->odom->getAngular();
  this->odomMsg.pose.pose.position.x = this->odom->getX();
  this->odomMsg.pose.pose.position.y = this->odom->getY();
  tf2::Quaternion yaw;
  yaw.setRPY(0, 0, this->odom->getHeading());
  tf2::convert(yaw, this->odomMsg.pose.pose.orientation);

  const auto dt = stamp - this->lastMeasurementTime;
  const auto dtSec = dt.toSec();
  const auto vx = this->odom->getLinear() * std::cos(this->odom->getHeading());
  const auto vy = this->odom->getLinear() * std::sin(this->odom->getHeading());

  const auto dx = vx * dtSec;
  const auto dy = vy * dtSec;

  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> poseCov(this->odomMsg.pose.covariance.data());
  const auto twistCovRotated = tf2::transformCovariance(this->odomMsg.twist.covariance, tf2::Transform(yaw));
  Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> twistCov(twistCovRotated.data());

  // This is a simple Jacobian that just projects yaw influence on X/Y motion in odom frame
  Eigen::Matrix<double, 6, 6> poseJacobian;
  // TODO figure out why abs() is needed
  poseJacobian <<
    1, 0, 0, 0, 0, -std::abs(dy),
    0, 1, 0, 0, 0, std::abs(dx),
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1;

  poseCov = poseJacobian * poseCov * poseJacobian.transpose() + twistCov;

  // limit angular covariance to (2*PI)^2
  for (Eigen::Index i = 3; i <= 5; ++i)
  {
    poseCov(i, i) = (std::min)(poseCov(i, i), 4 * M_PI * M_PI);
  }

  this->pub.publish(this->odomMsg);

  this->lastMsg = msg;
  this->lastTime = ros::Time::now();
  this->lastMeasurementTime = stamp;
}

void OpenLoopWheelOdom::reset()
{
  this->initialized = false;
  this->odom->reset();
  std::fill(this->odomMsg.pose.covariance.begin(), this->odomMsg.pose.covariance.end(), 0);
  for (size_t i = 0; i < 6; ++i)
  {
    this->odomMsg.pose.covariance[i + i * 6] = this->initialPoseCovDiag[i];
  }
  ROS_WARN("Resetting odometry to zero.");
}

void OpenLoopWheelOdom::onReset(const topic_tools::ShapeShifter& msg)
{
  this->reset();
}

}