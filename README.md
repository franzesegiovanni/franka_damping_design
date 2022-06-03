# FrankaAdvancedControllers
Extra Controllers for Control of Franka Emika Panda

To make it working with the simulator, please add: 
1. in franka_gazebo/CMakeList.txt  
in catkin_package(...)
add
```franka_advanced_controllers```

2. in franka_gazebo/CMakeList.txt  
in find_package(..)
add 
```franka_advanced_controllers```

3. in franka_gazebo/package.xml
add
```
<depend>franka_advanced_controllers</depend>
```
4. in franka_gazebo/config/sim_controllers.yaml
add
```
cartesian_impedance_advanced_controller:
  type: franka_advanced_controllers/CartesianImpedanceAdvancedController 
  arm_id: $(arg arm_id)
  joint_names:
    - $(arg arm_id)_joint1
    - $(arg arm_id)_joint2
    - $(arg arm_id)_joint3
    - $(arg arm_id)_joint4
    - $(arg arm_id)_joint5
    - $(arg arm_id)_joint6
    - $(arg arm_id)_joint7
```
If the robot stops during interaction because of reflex, go to change this parameters:
in /src/franka_ros/franka_control/config/franka_control_node.yaml

#Configure the initial defaults for the collision behavior reflexes.
collision_config:
  lower_torque_thresholds_acceleration: [50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0]  # [Nm]
  
  upper_torque_thresholds_acceleration: [50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0]  # [Nm]
  
  lower_torque_thresholds_nominal: [50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0]  # [Nm]
  
  upper_torque_thresholds_nominal: [50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0]  # [Nm]
  
  lower_force_thresholds_acceleration: [50.0, 50.0, 50.0, 50.0, 50.0, 50.0]  # [N, N, N, Nm, Nm, Nm]
  
  upper_force_thresholds_acceleration: [50.0, 50.0, 50.0, 50.0, 50.0, 50.0]  # [N, N, N, Nm, Nm, Nm] 
  
  lower_force_thresholds_nominal: [50.0, 50.0, 50.0, 50.0, 50.0, 50.0]  # [N, N, N, Nm, Nm, Nm]
  
  upper_force_thresholds_nominal: [50.0, 50.0, 50.0, 50.0, 50.0, 50.0]  # [N, N, N, Nm, Nm, Nm]
