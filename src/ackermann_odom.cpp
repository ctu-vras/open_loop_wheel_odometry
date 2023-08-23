// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Implementation of the open-loop odometry via ackermann_steering_controller's Odometry class.
 * \author Martin Pecka
 */

#include <ackermann_steering_controller/odometry.h>
#include <ros/ros.h>

#include <open_loop_wheel_odometry/open_loop_wheel_odometry.h>
#include <open_loop_wheel_odometry/ackermann_odom.h>

namespace open_loop_wheel_odometry
{

AckermannOdometry::AckermannOdometry()
{
  this->odom = std::make_unique<ackermann_steering_controller::Odometry>();
}

AckermannOdometry::~AckermannOdometry() // NOLINT(modernize-use-equals-default)
{
}

void AckermannOdometry::init(const ros::Time& time)
{
  this->odom->init(time);
}

void AckermannOdometry::updateOpenLoop(double linear, double angular, const ros::Time& time)
{
  this->odom->updateOpenLoop(linear, angular, time);
}

double AckermannOdometry::getLinear() const
{
  return this->odom->getLinear();
}

double AckermannOdometry::getAngular() const
{
  return this->odom->getAngular();
}

double AckermannOdometry::getX() const
{
  return this->odom->getX();
}

double AckermannOdometry::getY() const
{
  return this->odom->getY();
}

double AckermannOdometry::getHeading() const
{
  return this->odom->getHeading();
}

void AckermannOdometry::reset()
{
  this->odom = std::make_unique<ackermann_steering_controller::Odometry>();
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ackermann_odom");

  using namespace open_loop_wheel_odometry;
  std::unique_ptr<Odometry> ackermannOdom = std::make_unique<AckermannOdometry>();
  OpenLoopWheelOdom odom(ros::NodeHandle(), ros::NodeHandle("~"), std::move(ackermannOdom));

  ros::spin();
}