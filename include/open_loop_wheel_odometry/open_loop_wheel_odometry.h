#pragma once

/**
 * \file
 * \brief A generalized node that receives cmd_vel commands and integrates them via an open-loop odometry model.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <array>
#include <memory>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

#include <open_loop_wheel_odometry/odometry.h>

namespace open_loop_wheel_odometry
{

/**
 * \brief ROS node computing a open-loop wheeled vehicle odometry model and publishing it as an Odometry message.
 */
class OpenLoopWheelOdom
{
public:
  /**
   * \brief Create and start the node.
   * \note The calling code should take care of ROS spinning. This class does not spin any queue itself. 
   * \param[in] nodeHandle Public node handle (for topics).
   * \param[in] privateNodeHandle Private node handle (for parameters).
   * \param[in] odom An implementation of the odometry model.
   */
  OpenLoopWheelOdom(ros::NodeHandle nodeHandle, ros::NodeHandle privateNodeHandle, std::unique_ptr<Odometry> odom);
  
  virtual ~OpenLoopWheelOdom();

protected:
  /**
   * \brief Callback for cmd_vel messages.
   * \param[in] msg The commanded velocity. 
   */
  void onCmd(const topic_tools::ShapeShifter& msg);
  /**
   * \brief Callback for reset messages.
   * \param[in] msg Ignored. 
   */
  void onReset(const topic_tools::ShapeShifter& msg);

  /**
   * \brief Update the odometry model.
   * \param[in] msg The commanded velocity. 
   * \param[in] stamp Timestamp of the measurement. 
   */
  void update(const geometry_msgs::Twist& msg, const ros::Time& stamp);
  
  /**
   * \brief Reset the odometry to its initial state.
   */
  void reset();
  
  //! \brief An implementation of the odometry model.
  std::unique_ptr<Odometry> odom;
  
  //! \brief The odometry message that is being published. In constructor, we pre-fill some of its fields.
  nav_msgs::Odometry odomMsg;
  
  //! \brief Publisher of the odometry messages.
  ros::Publisher pub;
  
  //! \brief Subscriber to one of the cmd_vel type topics.
  ros::Subscriber sub;
  ros::Subscriber resetSub;
  
  //! \brief Whether at least one cmd_vel message has been processed.
  bool initialized {false};
  
  //! \brief Last processed message.
  geometry_msgs::Twist lastMsg;
  
  //! \brief Timestamp of the last processed measurement.
  ros::Time lastMeasurementTime;
  
  //! \brief Last time when we processed a message.
  ros::Time lastTime;
  
  //! \brief The diagonal elements of the initial pose covariance matrix.
  std::array<double, 6> initialPoseCovDiag {};
};

}