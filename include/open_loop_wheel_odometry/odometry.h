#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Definition of the interface of an odometry implementation.
 * \author Martin Pecka
 */

#include <ros/time.h>

namespace open_loop_wheel_odometry
{

/**
 * \brief A wheeled vehicle odometry.
 */
class Odometry
{
public:
  virtual ~Odometry() = default;

  /**
   * \brief Initialize the odometry in the given time.
   * \param[in] time The time to treat as odometry computation start.
   */
  virtual void init(const ros::Time& time) = 0;
  
  /**
   * \brief Perform the open-loop update of the odometry.
   * \param[in] linear Linear forward velocity in body frame [m/s]. 
   * \param[in] angular Angular yaw velocity in body frame [rad/s].
   * \param[in] time Time of the measurement.
   */
  virtual void updateOpenLoop(double linear, double angular, const ros::Time& time) = 0;
  
  /**
   * \brief Get the computed linear forward velocity.
   * \return Velocity in body frame [m/s].
   */
  virtual double getLinear() const = 0;
  
  /**
   * \brief Get the computed angular yaw velocity.
   * \return Yaw velocity in body frame [rad/s].
   */
  virtual double getAngular() const = 0;
  
  /**
   * \brief Get the computed X position of the vehicle.
   * \return The X position in odom frame [m].
   */
  virtual double getX() const = 0;

  /**
   * \brief Get the computed Y position of the vehicle.
   * \return The Y position in odom frame [m].
   */
  virtual double getY() const = 0;

  /**
   * \brief Get the computed yaw of the vehicle.
   * \return The yaw in odom frame [rad].
   */
  virtual double getHeading() const = 0;

  /**
   * \brief Reset the odometry (start on zero position again, reset the accumulated velocities etc.).
   */
  virtual void reset() = 0;
};

}