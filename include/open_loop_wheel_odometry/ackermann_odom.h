#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Implementation of the open-loop odometry via ackermann_steering_controller's Odometry class.
 * \author Martin Pecka
 */

#include <memory>

#include <ackermann_steering_controller/odometry.h>
#include <ros/time.h>

#include <open_loop_wheel_odometry/odometry.h>

namespace open_loop_wheel_odometry
{

/**
 * \brief Open-loop odometry of an Ackermann steering vehicle.
 */
class AckermannOdometry : public Odometry
{
public:
  AckermannOdometry();
  ~AckermannOdometry() override;
  void init(const ros::Time& time) override;
  void updateOpenLoop(double linear, double angular, const ros::Time& time) override;
  double getLinear() const override;
  double getAngular() const override;
  double getX() const override;
  double getY() const override;
  double getHeading() const override;
  void reset() override;

protected:
  //! \brief The ackermann_steering_controller implementation
  std::unique_ptr<ackermann_steering_controller::Odometry> odom;
};

}