![attachment:trapezoid.png](doc/trapezoid.png) Image source: Herman
Bruyninckx, Motion Interpolation

## joint_trajectory_generator

`joint_trajectory_generator` action takes in a trajectory specified by
a number of joint positions, and it generates a new smooth trajectory
through these joint positions. The smooth trajectory is created by
limiting the joint velocity and acceleration to specified maximal
values. The generated trajectory can have a longer duration than the
input trajectory if the input trajectory could only be executed with
joint velocities and accelerations that exceed the specified maximal
values. The generated trajectory will however never have a shorter
duration than the input trajectory.\
\
The action sends its output to the
[joint_trajectory_action](https://index.ros.org/p/joint_trajectory_action),
to execute the smooth trajectory it generated.

### Action Goal

- `joint_trajectory_generator/goal`
  ([pr2_controllers_msgs/msg/JointTrajectoryGoal](https://docs.ros.org/en/rolling/p/pr2_controllers_msgs/msg/JointTrajectoryGoal.html)):
  The goal contains the 'rough' trajectory that is only specified by a
  sequence of desired joint positions.

### Action Result

- `joint_trajectory_generator/result`
  ([pr2_controllers_msgs/msg/JointTrajectoryResult](https://docs.ros.org/en/rolling/p/pr2_controllers_msgs/msg/JointTrajectoryResult.html)):
  The result indicates if the trajectory was executed successfully.

### Actions Called

- `joint_trajectory_action`
  ([joint_trajectory_action](https://index.ros.org/p/joint_trajectory_action)):
  This action moves the arm to a joint configuration.

### Parameters

- `~max_acc` (`float64`): The maximum allowed acceleration of a joint.
  The generated trajectory will never make a joint accelerate faster
  than this value.
- `~/max_vel` (`float64`): The maximum allowed velocity of a joint. The
  generated trajectory will never make a joint move faster than this
  value.
- `robot_description` (`string`): The
  [urdf](https://index.ros.org/p/urdf) robot description. This is used
  to know if a joint is continuous or rotational.
