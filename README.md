# open\_loop\_wheel\_odometry

Open-loop odometry for diff-drive controller (and possibly others, too). This can be used to turn `cmd_vel` commands into odometry via a very simple no-feedback integration algorithm.

Currently, only diff-drive and Ackermann steering models are supported. This implementation is guaranteed to give the same results as `diff_drive_controller`/`ackermann_steering_controller` with `open_loop` parameter set to `True`.

More models can be added if needed.

This node correctly handles covariance (if you correctly specify the initial values). Especially, position covariance is growing without bounds.

## Common properties of all nodes

These properties are common to all implementations.

### Subscribed topics

- `cmd_vel_out` (`geometry_msgs/Twist` or `geometry_msgs/TwistStamped`): The cmd\_vel commands to integrate. Name of this topic corresponds to the `diff_drive_controller` velocity output topic, i.e. the commanded velocity after all limits are applied. It usually makes sense to compute the odometry from the real values sent to the motors and not from the desired values. Please note that the Ackermann steering controller doesn't have this output.
- `reset` (any type): Each time a message is received on this topic, the odometry is reset to its initial state.

### Published topics

- `odom_cmd_vel` (`nav_msgs/Odometry`): The integrated odometry. Please note that it is correct that the position covariance grows without bounds.

### Parameters

- `~odom_frame` (`str`, default `odom_cmd_vel`): The frame to use in `header.frame_id` of the published odometry messages.
- `~base_link_frame` (`str`, by default this frame is autodetected from the receieved messages): The frame to use in `child_frame_id` of the odometry messages. If this parameter is not specified and the incoming messages are either non-stamped or they have empty `frame_id`, an error message is printed and this parameter defaults to `base_link`.
- `~initial_pose_covariance_diagonal` (`list[float]`, default `1e-6, 1e-6, 1e-6, pi^2, pi^2, 1e-6`): Diagonal elements of the covariance of the initial state. This value has to be a 6-element list of floats, otherwise the default is used.
- `~twist_covariance_diagonal` (`list[float]`, default `1e-2, 1e-4, 1e-1, pi^2, pi^2, 0.5`): Diagonal elements of the covariance of the received velocity commands. Elements corresponding to the unmeasured quantities should not be very small.
- Please note there is no configuration of the diff-drive/Ackermann steering models. Their open-loop odometry just integrates the `cmd_vel` commands.

### Behavior

- The odometry automatically resets when using simulation time and the time jumps back by more than 3 seconds (this does not take into account the cmd\_vel message stamps, but actual ROS time).