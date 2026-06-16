# Foreman

***Note: Still under active development!***

ros2_control component lifecycle manager for easier brinup.

It takes a named group of component lifecycle states (a system state) in `config/scenario.yaml`, and tries to automatically transition between them.

## Quick start:

### 1. Describe your system

```yaml
controller_manager: my_controller_manager
transition_pause: 0.5

lifecycle_nodes:
  - my_node

hardware:
  - franka
  - kassow

controllers:
  joint_state_broadcaster:
    requires: [all, inactive]
  kassow_joint_trajectory_controller:
    requires:
      - [kassow, active]
      - [my_node, active]
  franka_joint_trajectory_controller:
    requires: [franka, active]

goal_states:

  idle:
    controllers:
      joint_state_broadcaster: inactive
      kassow_joint_trajectory_controller: inactive
      franka_joint_trajectory_controller: inactive
    hardware:
      franka: inactive
      kassow: inactive
    lifecycle_nodes:
      my_node: active

  # ... more named states
```

### 2. Run the node

```bash
ros2 run foreman foreman_node --ros-args -p config_path:=/path/to/scenario.yaml
```

### 3. Set the goal

```bash
ros2 service call /foreman/set_goal foreman_msgs/srv/SetGoal "{goal: 'idle'}"
```
... or any other named goal in `config/scenario.yaml`


### 4. Monitor the component state changes
```bash
ros2 topic echo /my_controller_manager/activity
```

## Architecture

Check `types.py` and `engine.py` for main API.
Check `planner.py` for business logic.
Check `adapters/` for how we integrate foreman with the rest of the system.
Check `node.py` how we glue it all up into a ros executable and run it.

- **Foreman Engine**
    - Pure python domain model
    - business logic + API, no ROS details
    - just take snapshot of the component states and output next state.

- **Adapters**
    - `ComponentStateMonitor` (inbound): Subscribes to `cm/activity` for HW + controllers. Monitors lifecycle nodes via `/node/transition_event` subscription with QoS matched event for discovery/death detection.
    - `ControllerManagerServiceCaller` (outbound): Calls `controller_manager` services.
    - `LifecycleNodeServiceCaller` (outbound): Calls `/node/change_state` for lifecycle node transitions.
    - `RosSetGoalServer` (inbound): Exposes `/foreman/set_goal` service.
    - `DatalayerAdapter` (outbound): ctrlX Datalayer integration (optional, to be added).
