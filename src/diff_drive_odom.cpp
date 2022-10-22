/**
 * \file
 * \brief Implementation of the open-loop odometry via diff_drive_controller's Odometry class.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <diff_drive_controller/odometry.h>
#include <ros/ros.h>

#include <open_loop_wheel_odometry/open_loop_wheel_odometry.h>
#include <open_loop_wheel_odometry/diff_drive_odom.h>

namespace open_loop_wheel_odometry
{

DiffDriveOdometry::DiffDriveOdometry()
{
  this->odom = std::make_unique<diff_drive_controller::Odometry>();
}

DiffDriveOdometry::~DiffDriveOdometry() // NOLINT(modernize-use-equals-default)
{
}

void DiffDriveOdometry::init(const ros::Time& time)
{
  this->odom->init(time);
}

void DiffDriveOdometry::updateOpenLoop(double linear, double angular, const ros::Time& time)
{
  this->odom->updateOpenLoop(linear, angular, time);
}

double DiffDriveOdometry::getLinear() const
{
  return this->odom->getLinear();
}

double DiffDriveOdometry::getAngular() const
{
  return this->odom->getAngular();
}

double DiffDriveOdometry::getX() const
{
  return this->odom->getX();
}

double DiffDriveOdometry::getY() const
{
  return this->odom->getY();
}

double DiffDriveOdometry::getHeading() const
{
  return this->odom->getHeading();
}

void DiffDriveOdometry::reset()
{
  this->odom = std::make_unique<diff_drive_controller::Odometry>();
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "diff_drive_odom");

  using namespace open_loop_wheel_odometry;
  std::unique_ptr<Odometry> diffDriveOdom = std::make_unique<DiffDriveOdometry>();
  OpenLoopWheelOdom odom(ros::NodeHandle(), ros::NodeHandle("~"), std::move(diffDriveOdom));

  ros::spin();
}