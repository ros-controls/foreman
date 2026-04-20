# Foreman

ros2_control component lifecycle manager for easier brinup.

It takes a named group of component lifecycle states (a system state) in `config/scenario.yaml`, and tries to automatically transition between them.

## Quick start:

### 1. Describe your system

```yaml
controller_manager: my_controller_manager
transition_pause: 0.5

hardware:
  - franka
  - kassow

controllers:
  joint_state_broadcaster:
    requires: [all, inactive]
  kassow_joint_trajectory_controller:
    requires: [kassow, active]
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
  
  # ... more named states
```

### 2. Run the node

```bash
ros2 run foreman foreman_node
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
    - to `controller_manager`:
        - `StateMonitor`: parse `/activity` into system state
        - `ServiceCaller`: triggers `controller_manager` services based on planner output
    - to `ROS`:
        - `SetGoalServer`: exposes a ROS service to set goal state
    - to `Datalayer`
        - `DatalayerClient`: **To be added**, listens to Datalayer events to set goal state
