# Target reach with terminal yaw

The Unipilot profile accepts a position and terminal heading on the existing `/move_base_simple/goal` `geometry_msgs/PoseStamped` interface. GBPlanner keeps its path planning and positional reaching radius. Once within that radius, it commands a stationary turn until heading error is within `PlanningParams/local_navigation_yaw_tolerance`. A valid quaternion requests its yaw; an all-zero/invalid quaternion retains positional-only behavior. An RViz 2D Nav Goal therefore also requests the arrow's heading.

PCI preserves stationary rotation segments and applies `path_end_yaw_thr` to the terminal waypoint. NMPC must use `ref.yaw_mode: ref` to follow waypoint orientations; the Unipilot profile already does. Wrapped angular differences use the shortest turn across ±pi. No additional ROS messages or bridge entries are needed.

## Agent configuration

Set in `robot_bringup/config/ros2/agentic_uas_unipilot.yaml`:

```yaml
enable_goal_yaw: true
goal_yaw_tolerance: 0.2
```

This applies to all six methods. Move responses include `yaw_offset` in radians in `[-pi, pi]`, relative to robot heading when inference starts. Positive is counterclockwise about world +Z: `+pi/2` faces left, `-pi/2` faces right. The agent supplies heading-reference vectors in the current optical frame, preserving local-image methods' input convention. Prefetched targets retain that frozen reference heading. Finish/wait responses use null yaw.

The default `enable_goal_yaw: false` does not request a model heading; the goal uses the robot's current heading when published. The agent waits for both positional and angular completion, and angular movement counts as progress. Status includes `goal_yaw` and `goal_yaw_error`.

Keep agent `goal_yaw_tolerance`, planner `local_navigation_yaw_tolerance` and PCI `path_end_yaw_thr` aligned; the Unipilot profile uses 0.2 radians. Other controller profiles need their own reference-heading mode and tolerance verification.

## Execution check

Build the affected GBPlanner/PCI, agent/bringup workspaces. With the Unipilot simulation running, execute the root check script inside a Humble container with the matching ROS domain and workspace sourced:

```bash
python3 /path/to/scripts/check_target_reach_yaw.py --takeoff --output /tmp/target-yaw.json
```

The optional `--takeoff` initializes simulation flight only in the test harness. Without it, start from a hovering robot. The check stops the planner, sends a quaternion goal, switches to waypoint mode and starts planning through the existing bridged services. It checks terminal path orientation and actual robot yaw for in-place, ±pi wrap-boundary, and translated targets. `--translation 0` omits translation; choose an unobstructed target direction for your world. The script stops the planner on exit.

## Validation

Verified in the office Unipilot simulation through ROS 2 -> ROS 1 goal/service bridges, rebuilt GBPlanner/PCI, NMPC and Gazebo odometry. Stationary goals at +pi/2, +3.05 and -3.05 radians reached <0.2 rad error. A goal 4 m along global x reached the existing 2 m positional radius with 0.145 rad yaw error. Intermediate planning segments retain their navigation heading; terminal heading is asserted only on the segment reaching the target region. Measurements and logs are saved in `evaluation_results/target-reach-yaw-validation/`.

126 agent tests and 5 NMPC reference tests pass, including all six method schemas, enabled/disabled publication, heading-aware arrival, frozen prefetch reference, zero-distance segments and angular wrap. ROS 1 planner/PCI and ROS 2 agent/bringup builds pass. Other robot profiles and hardware have not been tested.
