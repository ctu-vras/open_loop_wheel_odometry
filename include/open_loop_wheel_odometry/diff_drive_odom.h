#pragma once

/**
 * \file
 * \brief Implementation of the open-loop odometry via diff_drive_controller's Odometry class.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>

#include <diff_drive_controller/odometry.h>
#include <ros/time.h>

#include <open_loop_wheel_odometry/odometry.h>

namespace open_loop_wheel_odometry
{

/**
 * \brief Open-loop odometry of a diff-drive vehicle.
 */
class DiffDriveOdometry : public Odometry
{
public:
  DiffDriveOdometry();
  ~DiffDriveOdometry() override;
  void init(const ros::Time& time) override;
  void updateOpenLoop(double linear, double angular, const ros::Time& time) override;
  double getLinear() const override;
  double getAngular() const override;
  double getX() const override;
  double getY() const override;
  double getHeading() const override;
  void reset() override;

protected:
  //! \brief The diff_drive_controller implementation
  std::unique_ptr<diff_drive_controller::Odometry> odom;
};

}