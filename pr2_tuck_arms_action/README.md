![attachment:tuckarm.jpg](doc/tuckarm.jpg)

## pr2_tuck_arm_action

`pr2_tuck_arm_action` action uses the
[joint_trajectory_action](https://index.ros.org/p/joint_trajectory_action)
to tuck and untuck the PR2's arms. Tucked arms allow the PR2 to
navigate without obstructing its own view.

### Action Goal

- `tuck_arms/goal`
  ([pr2_common_action_msgs/msg/TuckArmsGoal](https://docs.ros.org/en/rolling/p/pr2_common_action_msgs/msg/TuckArmsGoal.html)):
  This message specifies with arm(s) to (un)tuck.

### Action Result

- `tuck_arms/result`
  ([pr2_common_action_msgs/msg/TuckArmsResult](https://docs.ros.org/en/rolling/p/pr2_common_action_msgs/msg/TuckArmsResult.html)):
  Empty result message. The action status provides information on
  success/failure.

### Actions Called

- `r_arm_controller/<~joint_trajectory_action> and l_arm_controller/<~joint_trajectory_action>`
  ([joint_trajectory_action](https://index.ros.org/p/joint_trajectory_action)):
  This action moves the arm to a joint configuration.

### Parameters

- `~r_joint_trajectory_action` (`string`, default:
  `r_arm_controller/joint_trajectory_action`): The namespace of the
  right arm joint trajectory action to send trajectories to.
- `~l_joint_trajectory_action` (`string`, default:
  `l_arm_controller/joint_trajectory_action`): The namespace of the left
  arm joint trajectory action to send trajectories to.
- `~move_duration` (`float`, default: `2.5 seconds`): The duration of
  each trajectory segment that is executed by tuck arms. If you use a
  trajectory generator, you can set this value to zero to achieve the
  maximum possible speed.

### Examples

For a running ROS system with a PR2 robot the following terminal command
can be written to tuck the arms
` rosrun pr2_tuck_arms_action tuck_arms.py -lt `

See [tucking arms video](http://vimeo.com/57923959)
